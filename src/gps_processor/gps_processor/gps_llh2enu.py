import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class GpsToEnuConverter(Node):
    def __init__(self):
        super().__init__('gps_llh2enu')
        
        # WGS84 타원체 상수 (단위: m)
        self.a = 6378137.0
        self.b = 6356752.314245
        
        # 기준점 저장 변수
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        
        # 미리 계산해둘 곡률 반경 상수
        self.M_ref = None
        self.N_ref = None

        # 1. Rviz2 시각화를 위한 Path 퍼블리셔 생성
        self.path_pub = self.create_publisher(Path, '/gps_path', 10)
        
        # 누적할 궤적 메시지 초기화
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map' # Rviz2에서 기준이 될 프레임 이름

        # /f9p/fix 토픽 구독
        self.subscription = self.create_subscription(
            NavSatFix,
            '/f9p/fix',
            self.listener_callback,
            10
        )
        self.get_logger().info('GPS to ENU 변환 노드가 시작되었습니다. 첫 GPS 신호를 대기 중입니다...')

    def calculate_meridional_radius(self, lat_rad):
        # 자오선 곡률 반경 계산
        return ((self.a * self.b) ** 2) / (((self.a * math.cos(lat_rad)) ** 2) + ((self.b * math.sin(lat_rad)) ** 2)) ** 1.5

    def calculate_normal_radius(self, lat_rad):
        # 횡단 곡률 반경 계산
        return (self.a ** 2) / math.sqrt(((self.a * math.cos(lat_rad)) ** 2) + ((self.b * math.sin(lat_rad)) ** 2))

    def listener_callback(self, msg):
        # 유효하지 않은 GPS 데이터는 무시
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)
        alt = msg.altitude

        # 첫 수신 시 기준점(Reference Point) 설정 및 상수 계산
        if self.ref_lat is None:
            self.ref_lat = lat_rad
            self.ref_lon = lon_rad
            self.ref_alt = alt
            
            self.M_ref = self.calculate_meridional_radius(self.ref_lat)
            self.N_ref = self.calculate_normal_radius(self.ref_lat)
            
            self.get_logger().info(f'✅ 기준점(Origin)이 설정되었습니다.')
            self.get_logger().info(f'  - 위경도: {msg.latitude:.6f}, {msg.longitude:.6f}')
            self.get_logger().info(f'  - 반경 상수 M: {self.M_ref:.2f}m, N: {self.N_ref:.2f}m')
            return

        # 현재 위치와 기준점의 차이
        delta_lat = lat_rad - self.ref_lat
        delta_lon = lon_rad - self.ref_lon
        delta_alt = alt - self.ref_alt

        # Simplified Coordinate Conversion 공식을 이용한 ENU 변환
        east = (self.N_ref + delta_alt) * math.cos(self.ref_lat) * delta_lon
        north = (self.M_ref + delta_alt) * delta_lat
        up = delta_alt

        # 2. 현재 위치를 PoseStamped 메시지로 생성
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        
        # X: East, Y: North, Z: Up
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = up
        
        # 방향(Orientation)은 기본값 설정 (단순 위치 추적이므로 회전 없음)
        pose.pose.orientation.w = 1.0 

        # 3. Path 메시지에 현재 위치 추가 및 퍼블리시
        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        
        self.path_pub.publish(self.path_msg)

        # 결과 출력
        self.get_logger().info(f'📍 ENU 좌표 (m) -> East(X): {east: .3f}, North(Y): {north: .3f}, Up(Z): {up: .3f}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsToEnuConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 안전하게 종료되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()