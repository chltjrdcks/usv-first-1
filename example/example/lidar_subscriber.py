import math
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan


class LidarImuSubscriber(Node):

  def __init__(self):
    super().__init__('lidar_subscriber')

    # 서버 주소 설정 (스프링부트)
    self.server_url = 'http://localhost:8080/actuator/key/degree'

    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    # 1. LiDAR 서브스크라이버 (/scan)
    self.scan_sub = self.create_subscription(
        LaserScan, '/scan', self.scan_callback, qos_profile
    )

    # 2. IMU 서브스크라이버 (/imu)
    self.imu_sub = self.create_subscription(
        Imu, '/imu', self.imu_callback, qos_profile
    )

    # IMU 오일러 각도 변수 (X: Roll, Y: Pitch, Z: Yaw) - degree 단위
    self.roll_deg = 0.0
    self.pitch_deg = 0.0
    self.yaw_deg = 0.0

  def euler_from_quaternion(self, x, y, z, w):
    """Quaternion (x, y, z, w) -> Euler Angles Roll(X), Pitch(Y), Yaw(Z) 변환 (라디안)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
      pitch = math.copysign(math.pi / 2.0, sinp)
    else:
      pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

  def imu_callback(self, msg):
    q = msg.orientation

    # 쿼터니언 -> 라디안 변환
    roll_rad, pitch_rad, yaw_rad = self.euler_from_quaternion(
        q.x, q.y, q.z, q.w
    )

    # 라디안 -> 도(Degree) 변환
    self.roll_deg = round(math.degrees(roll_rad), 2)
    self.pitch_deg = round(math.degrees(pitch_rad), 2)
    self.yaw_deg = round(math.degrees(yaw_rad), 2)

    self.get_logger().info(
        f'IMU Angles -> Roll(X): {self.roll_deg}° | Pitch(Y):'
        f' {self.pitch_deg}° | Yaw(Z): {self.yaw_deg}°'
    )

  def scan_callback(self, msg):
    if len(msg.ranges) == 0:
      return

    min_distance = float('inf')
    min_index = 0

    # 가장 가까운 장애물 탐색
    for i, distance in enumerate(msg.ranges):
      if msg.range_min < distance < msg.range_max:
        if distance < min_distance:
          min_distance = distance
          min_index = i

    if min_distance == float('inf'):
      return

    # 장애물 수평 상대 각도 계산 (LiDAR 기준)
    angle_rad = msg.angle_min + (min_index * msg.angle_increment)
    angle_deg = math.degrees(angle_rad)

    # -180 ~ 180도 정규화
    while angle_deg > 180.0:
      angle_deg -= 360.0
    while angle_deg < -180.0:
      angle_deg += 360.0

    target_degree = round(-angle_deg)

    # 서버로 라이다 각도와 IMU X, Y, Z 각도를 모두 포함하여 전송
    self.send_data_to_actuator(
        target_degree=target_degree,
        roll=self.roll_deg,
        pitch=self.pitch_deg,
        yaw=self.yaw_deg,
    )

  def send_data_to_actuator(self, target_degree, roll, pitch, yaw):
    payload = {
        'degree': target_degree,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw,
    }
    try:
      requests.post(self.server_url, json=payload, timeout=0.1)
    except requests.exceptions.RequestException:
      pass


def main(args=None):
  rclpy.init(args=args)
  node = LidarImuSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()