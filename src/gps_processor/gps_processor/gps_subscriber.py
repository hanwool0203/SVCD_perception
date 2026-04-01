import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        
        # /f9p/fix 토픽을 구독 (메시지 타입: NavSatFix, 큐 사이즈: 10)
        self.subscription = self.create_subscription(
            NavSatFix,
            '/f9p/fix',
            self.listener_callback,
            10
        )
        self.subscription  # 경고 방지용

    def listener_callback(self, msg):
        # 실시간으로 들어오는 메시지에서 위도, 경도, 고도 값을 변수에 담습니다.
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        # 터미널에 보기 좋게 출력합니다.
        self.get_logger().info(f'📍 수신된 GPS 데이터 -> 위도: {lat:.7f}, 경도: {lon:.7f}, 고도: {alt:.3f}m')

def main(args=None):
    rclpy.init(args=args)
    
    # 노드 생성 및 실행
    gps_subscriber = GpsSubscriber()
    
    try:
        rclpy.spin(gps_subscriber)
    except KeyboardInterrupt:
        gps_subscriber.get_logger().info('GPS 구독 노드가 종료되었습니다.')
    finally:
        # 종료 시 메모리 정리
        gps_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()