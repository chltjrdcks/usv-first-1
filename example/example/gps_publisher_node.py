import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsPublisherNode(Node):

  def __init__(self):
    super().__init__('gps_publisher_node')

    # /gps/fix 토픽으로 퍼블리셔 생성
    self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

    # 1초에 한 번씩 콜백 함수 실행 (1Hz)
    self.timer_period = 1
    self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # 기준 시작 위치 (위도, 경도)
    self.latitude = 35.1958165
    self.longitude = 129.0780031

    # 이동 속도 설정 (초당 3m)
    self.speed_m_per_sec = 1.0

    # 위도 35.1958165도 위치에서의 경도 1도당 거리 계산 (지구 위도별 위선 반지름 반영)
    # R_earth * cos(lat) * (pi / 180)
    lat_rad = math.radians(self.latitude)
    earth_radius_m = 6378137.0  # WGS84 지구 적도 반지름
    meters_per_degree_lon = earth_radius_m * math.cos(lat_rad) * (math.pi / 180.0)

    # 1초당(3m) 증가할 경도 변화량 (degree)
    self.lon_increment_per_sec = self.speed_m_per_sec / meters_per_degree_lon

    self.get_logger().info('GPS Publisher Node Started!')
    self.get_logger().info(
        f'Initial Position -> Lat: {self.latitude}, Lon: {self.longitude}'
    )
    self.get_logger().info(f'Moving East at {self.speed_m_per_sec} m/s...')

  def timer_callback(self):
    msg = NavSatFix()

    # 메시지 헤더 설정
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'gps_link'

    # GPS 상태 설정 (정상 수신 상태)
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS

    # 현재 좌표 대입
    msg.latitude = self.latitude
    msg.longitude = self.longitude
    msg.altitude = 0.0  # 해수면 고도 (기본값 0m)

    # 위치 공분산 (기본 정밀도 설정)
    msg.position_covariance = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    # 토픽 발행
    self.publisher_.publish(msg)

    self.get_logger().info(
        f'Published GPS -> Lat: {msg.latitude:.7f}, Lon: {msg.longitude:.7f}'
    )

    # 동쪽으로 이동 (경도 증가)
    self.longitude += self.lon_increment_per_sec


def main(args=None):
  rclpy.init(args=args)
  node = GpsPublisherNode()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()