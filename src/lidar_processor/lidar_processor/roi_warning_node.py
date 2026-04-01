#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
import yaml
import os

class RoiWarningNode(Node):
    def __init__(self):
        super().__init__('roi_warning_node')

        # -----------------------------------------------------------
        # 1. 설정 파일 절대 경로 지정
        # -----------------------------------------------------------
        # 사용자 환경에 맞춰 경로를 지정합니다. (os.path.expanduser로 ~ 틸드 처리)
        self.yaml_path = os.path.expanduser('~/workspace/grad_ws/src/lidar_processor/cfg/roi.yaml')
        self.last_mtime = 0.0 # 파일의 마지막 수정 시간 저장용

        # 기본값 (파일이 없거나 오류가 났을 때 사용할 안전한 기본 구역)
        self.frame_id = 'velodyne'
        self.roi = {
            'x_min': 0.0, 'x_max': 10.0,
            'y_min': -2.0, 'y_max': 2.0,
            'z_min': -1.5, 'z_max': 1.0
        }

        # 초기 설정 로드
        self.load_yaml_config()

        # -----------------------------------------------------------
        # 2. 통신 (Sub / Pub)
        # -----------------------------------------------------------
        self.obj_sub = self.create_subscription(
            MarkerArray, '/detected_objects', self.object_callback, 10
        )
        self.warning_pub = self.create_publisher(Bool, '/roi_warning', 10)
        self.roi_vis_pub = self.create_publisher(Marker, '/roi_visualization', 10)

        self.get_logger().info("🚨 ROI Warning Node Started! (Auto-Reload Enabled)")

    def load_yaml_config(self):
        """YAML 파일이 수정되었는지 확인하고, 수정되었다면 새로 불러옵니다."""
        if not os.path.exists(self.yaml_path):
            return # 파일이 없으면 기본값 유지

        # 파일의 마지막 수정 시간(Modified Time) 확인
        current_mtime = os.path.getmtime(self.yaml_path)
        
        # 이전 수정 시간과 다르다면 = "파일이 저장되었다면"
        if current_mtime != self.last_mtime:
            try:
                with open(self.yaml_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                
                if config:
                    self.frame_id = config.get('frame_id', self.frame_id)
                    yaml_roi = config.get('roi', {})
                    # 기존 딕셔너리에 새 값을 덮어씌움
                    self.roi.update(yaml_roi)

                self.last_mtime = current_mtime
                self.get_logger().info(f"🔄 ROI 설정 파일이 실시간 업데이트 되었습니다! (x_max: {self.roi['x_max']})")
                
            except Exception as e:
                self.get_logger().error(f"❌ YAML 파싱 에러 (문법을 확인하세요): {e}")

    def object_callback(self, msg: MarkerArray):
        # 1. 매 프레임마다 파일 수정 여부를 0.0001초 만에 확인합니다.
        self.load_yaml_config()

        is_dangerous = False

        target_ids = set()
        for marker in msg.markers:
            if marker.ns == "info":
                target_classes = ["Car", "Ped", "Moto", "Bike"]
                if any(cls in marker.text for cls in target_classes):
                    original_id = marker.id - 10000
                    target_ids.add(original_id)
        # 2. ROI 검사
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue

            # 실제 박스 형태인 마커(objects)이면서, ID가 타겟(Car/Ped)인 경우에만 검사
            if marker.ns == "objects" and marker.id in target_ids:
                obj_x = marker.pose.position.x
                obj_y = marker.pose.position.y
                obj_z = marker.pose.position.z

                if (self.roi['x_min'] <= obj_x <= self.roi['x_max']) and \
                   (self.roi['y_min'] <= obj_y <= self.roi['y_max']) and \
                   (self.roi['z_min'] <= obj_z <= self.roi['z_max']):
                    is_dangerous = True
                    break # 위험 객체를 하나라도 찾으면 더 검사할 필요 없음

        # 3. 경고 메시지 발행
        warning_msg = Bool()
        warning_msg.data = is_dangerous
        self.warning_pub.publish(warning_msg)

        if is_dangerous:
            # throttle_duration_sec=1.0 : 1초에 한 번만 출력하여 터미널 도배 방지
            self.get_logger().warn(
                "🛑 [정지 명령] ROI 영역 내에 위험 객체(차량/보행자/이륜차)가 감지되었습니다!", 
                throttle_duration_sec=1.0
            )

        # 4. ROI 박스 시각화 발행
        self.publish_roi_marker(is_dangerous)

    def publish_roi_marker(self, is_dangerous):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "roi_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # ROI 박스의 중심 좌표 계산
        marker.pose.position.x = (self.roi['x_max'] + self.roi['x_min']) / 2.0
        marker.pose.position.y = (self.roi['y_max'] + self.roi['y_min']) / 2.0
        marker.pose.position.z = (self.roi['z_max'] + self.roi['z_min']) / 2.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # ROI 박스의 크기(Scale) 계산
        marker.scale.x = float(self.roi['x_max'] - self.roi['x_min'])
        marker.scale.y = float(self.roi['y_max'] - self.roi['y_min'])
        marker.scale.z = float(self.roi['z_max'] - self.roi['z_min'])

        # 색상 설정 (빨강/초록)
        marker.color.a = 0.3
        if is_dangerous:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0

        self.roi_vis_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RoiWarningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()