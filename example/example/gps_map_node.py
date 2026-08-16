import math
import json
import asyncio
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64

from aiohttp import web

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>ROS 2 GPS & Autonav System</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        body { margin: 0; padding: 0; font-family: sans-serif; }
        #map { height: 100vh; width: 100vw; cursor: crosshair; }
        #info-panel {
            position: absolute; top: 15px; left: 50px; z-index: 1000;
            background: rgba(255, 255, 255, 0.92); padding: 15px 20px;
            border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.3);
            font-size: 13px; line-height: 1.6; min-width: 240px;
        }
        .highlight { color: #0056b3; font-weight: bold; }
        .target { color: #d9534f; font-weight: bold; }
    </style>
</head>
<body>
    <div id="info-panel">
        <b>🛰️ 실시간 상태</b><br>
        • 현재 GPS: <span id="current-gps" class="highlight">-</span><br>
        • IMU Heading: <span id="imu-heading" class="highlight">-°</span><br>
        <hr style="margin: 8px 0;">
        <b>🎯 목적지 제어</b><br>
        • 마우스 좌표: <span id="mouse-coords">-</span><br>
        • 선택 목적지: <span id="target-coords" class="target">미설정</span><br>
        • 목표 방위각: <span id="target-bearing" class="target">-°</span><br>
        • 서보 목표각: <span id="servo-angle" class="highlight">-°</span>
    </div>
    <div id="map"></div>

    <script>
        var map = L.map('map').setView([35.1958, 129.1056], 17);
        var robotMarker = null;
        var targetMarker = null;
        var pathPolyline = L.polyline([], {color: 'blue', weight: 3}).addTo(map);

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        var ws = new WebSocket("ws://" + location.host + "/ws");

        map.on('mousemove', function(e) {
            document.getElementById('mouse-coords').innerText = 
                e.latlng.lat.toFixed(6) + ", " + e.latlng.lng.toFixed(6);
        });

        map.on('click', function(e) {
            var lat = e.latlng.lat;
            var lng = e.latlng.lng;

            if (!targetMarker) {
                targetMarker = L.marker([lat, lng], {icon: L.divIcon({
                    className: 'custom-div-icon',
                    html: "<div style='background-color:#d9534f;width:12px;height:12px;border-radius:50%;border:2px solid white;'></div>",
                    iconSize: [15, 15]
                })}).addTo(map);
            } else {
                targetMarker.setLatLng([lat, lng]);
            }

            document.getElementById('target-coords').innerText = lat.toFixed(6) + ", " + lng.toFixed(6);
            ws.send(JSON.stringify({ type: 'set_target', lat: lat, lng: lng }));
        });

        ws.onmessage = function(event) {
            var data = JSON.parse(event.data);

            if (data.type === 'status_update') {
                var lat = data.lat;
                var lng = data.lng;

                document.getElementById('current-gps').innerText = lat.toFixed(6) + ", " + lng.toFixed(6);
                document.getElementById('imu-heading').innerText = data.heading.toFixed(1) + "°";
                document.getElementById('servo-angle').innerText = data.servo.toFixed(1) + "°";

                if (data.bearing !== null) {
                    document.getElementById('target-bearing').innerText = data.bearing.toFixed(1) + "°";
                }

                var newLatLng = new L.LatLng(lat, lng);

                if (!robotMarker) {
                    robotMarker = L.marker(newLatLng).addTo(map);
                    map.setView(newLatLng, 18);
                } else {
                    robotMarker.setLatLng(newLatLng);
                }
                pathPolyline.addLatLng(newLatLng);
            }
        };
    </script>
</body>
</html>
"""

class GpsAutonavNode(Node):
    def __init__(self):
        super().__init__('gps_map_node')

        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Publishers (서보 조향각 및 쓰러스터 출력)
        self.pub_servo = self.create_publisher(Float64, '/actuator/key/degree', 10)
        self.pub_thruster = self.create_publisher(Float64, '/actuator/thruster/percentage', 10)

        self.current_lat = None
        self.current_lng = None
        self.current_heading = 0.0

        self.target_lat = None
        self.target_lng = None
        self.target_bearing = None
        self.servo_angle = 0.0

        self.websockets = set()
        self.last_emit_time = self.get_clock().now()

        self.app = web.Application()
        self.app.router.add_get('/', self.handle_index)
        self.app.router.add_get('/ws', self.handle_websocket)

        self.get_logger().info('GPS Single Node 시작됨: http://localhost:8000')

    async def handle_index(self, request):
        return web.Response(text=HTML_TEMPLATE, content_type='text/html')

    async def handle_websocket(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.websockets.add(ws)

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get('type') == 'set_target':
                        self.target_lat = float(data['lat'])
                        self.target_lng = float(data['lng'])
                        self.calculate_and_control()
        finally:
            self.websockets.remove(ws)
        return ws

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        
        yaw_deg = math.degrees(yaw_rad)
        
        # 진북 기준 0° 정규화 (시계방향 +)
        # ROS 관례(동=0°, 반시계+)를 나침반 방위(북=0°, 시계+)로 변환
        heading_deg = (90.0 - yaw_deg) % 360.0
            
        self.current_heading = heading_deg
        self.calculate_and_control()

    def gps_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        self.current_lat = msg.latitude
        self.current_lng = msg.longitude
        
        self.calculate_and_control()
        self.broadcast_status()

    def calculate_and_control(self):
        if self.current_lat is None or self.target_lat is None:
            # [디버그] GPS/목표점 중 하나가 아직 없어서 계산을 건너뜀.
            # 이게 계속 찍히면 target_bearing이 한 번도 계산되지 않아
            # 화면엔 항상 0.0(디폴트 값)만 보이는 상태입니다.
            self.get_logger().warn(
                f'[BEARING DEBUG] 계산 스킵 - current_lat={self.current_lat}, target_lat={self.target_lat}'
            )
            return

        lat1 = math.radians(self.current_lat)
        lng1 = math.radians(self.current_lng)
        lat2 = math.radians(self.target_lat)
        lng2 = math.radians(self.target_lng)

        d_lng = lng2 - lng1

        y = math.sin(d_lng) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lng)

        bearing_rad = math.atan2(y, x)

        # atan2(y, x) 결과는 이미 "북=0°, 시계방향+" 나침반 기준으로 나온다.
        # (예: 적도상에서 정동쪽 목표 -> y>0, x=0 -> atan2=90° = 정동쪽 방위각과 일치)
        # 따라서 여기서 +90.0을 더하면 오히려 90°만큼 밀려서 잘못 계산된다. (기존 버그, 제거함)
        # %360.0만으로 음수 각도를 0~360 범위로 정규화하면 충분하다.
        bearing_deg = math.degrees(bearing_rad) % 360.0
        self.target_bearing = bearing_deg

        # 오차 각도 연산
        error_angle = self.target_bearing - self.current_heading

        # -180° ~ +180° 범위 정규화 (최단 거리 회전)
        error_angle = (error_angle + 180.0) % 360.0 - 180.0

        # 만약 실제 하드웨어 회전 방향이 반대(우회전해야 하는데 좌회전)라면 
        # 아래 식을 error_angle 대신 -error_angle 로 수정하시면 됩니다.
        self.servo_angle = error_angle

        # [디버그] 실제 좌표 / 계산 결과 확인용. 원인 파악되면 지워도 됩니다.
        self.get_logger().info(
            f'[BEARING DEBUG] cur=({self.current_lat:.6f}, {self.current_lng:.6f}) '
            f'target=({self.target_lat:.6f}, {self.target_lng:.6f}) '
            f'bearing={bearing_deg:.1f}° heading={self.current_heading:.1f}° '
            f'error={self.servo_angle:.1f}°'
        )

        # 1. 서보모터 조향각 발행
        servo_msg = Float64()
        servo_msg.data = float(self.servo_angle)
        self.pub_servo.publish(servo_msg)

        # 2. 쓰러스터 출력 상시 30% 발행
        thruster_msg = Float64()
        thruster_msg.data = 30.0
        self.pub_thruster.publish(thruster_msg)

    def broadcast_status(self):
        now = self.get_clock().now()
        if (now - self.last_emit_time).nanoseconds < 100_000_000:
            return
        self.last_emit_time = now

        if self.websockets and self.current_lat is not None:
            payload = json.dumps({
                'type': 'status_update',
                'lat': self.current_lat,
                'lng': self.current_lng,
                'heading': self.current_heading,
                'bearing': self.target_bearing if self.target_bearing is not None else 0.0,
                'servo': self.servo_angle
            })
            for ws in list(self.websockets):
                asyncio.run_coroutine_threadsafe(ws.send_str(payload), self.loop)

def start_web_server(node, loop):
    asyncio.set_event_loop(loop)
    node.loop = loop
    runner = web.AppRunner(node.app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, '0.0.0.0', 8000)
    loop.run_until_complete(site.start())
    loop.run_forever()

def main(args=None):
    rclpy.init(args=args)
    node = GpsAutonavNode()

    loop = asyncio.new_event_loop()
    web_thread = threading.Thread(target=start_web_server, args=(node, loop), daemon=True)
    web_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    # ㅎㅇ