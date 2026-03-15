#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2  # ROS2 공식 유틸


class PointCloudConverterNode(Node):
    def __init__(self):
        super().__init__('pc2np_node')

        self.frame_count = 0  # 디버깅용 카운터
        self.first_fields_logged = False

        # '/velodyne_points' 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10  # QoS depth
        )

        self.get_logger().info("LiDAR to Numpy 변환 노드 시작 (read_points 사용)...")
        self.get_logger().info("'/velodyne_points' 토픽을 기다리는 중...")

    def lidar_callback(self, msg: PointCloud2):
        """
        /velodyne_points 수신 시마다 호출
        PointCloud2 -> (N,4) numpy 배열 [x, y, z, intensity]
        """

        # 첫 프레임에서 필드 정보 한 번 찍어보기 (디버깅용)
        if not self.first_fields_logged:
            field_info = [(f.name, f.datatype, f.offset, f.count) for f in msg.fields]
            self.get_logger().info(f"PointCloud2 fields: {field_info}")
            self.first_fields_logged = True

        try:
            # 1. PointCloud2 -> generator of (x, y, z, intensity)
            #    여기서 각 값은 파이썬 float/int로 넘어오므로 dtype 섞여도 상관없음.
            points_iter = pc2.read_points(
                msg,
                field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=True
            )

            points_list = []
            for p in points_iter:
                # p: (x, y, z, intensity)
                # float()로 한 번 감싸면 타입이 뭐든 간에 float으로 통일됨
                x, y, z, intensity = p
                points_list.append([
                    float(x),
                    float(y),
                    float(z),
                    float(intensity)
                ])

            if len(points_list) == 0:
                self.get_logger().warn("받은 포인트가 0개입니다.")
                return

            # 2. (N,4) numpy 배열로 변환 + float32 캐스팅
            points_xyzi = np.array(points_list, dtype=np.float32)

            # 혹시라도 (4,)로 나올 경우를 대비한 안전장치
            if points_xyzi.ndim == 1:
                points_xyzi = points_xyzi.reshape(-1, 4)

        except Exception as e:
            self.get_logger().error(f"PointCloud2 -> numpy 변환 실패: {e}")
            return

        # 3. 디버깅용 로그: 5 frame마다 shape 출력
        if self.frame_count % 5 == 0:
            self.get_logger().info(
                f"[Frame {self.frame_count}] 변환 성공! points_xyzi shape = {points_xyzi.shape}"
            )

        self.frame_count += 1

        # TODO: 여기서 OpenPCDet 전처리/추론 호출
        # self.run_openpcdet_preprocessing(points_xyzi)
        # 예: ZeroMQ로 conda환경 pcdet 서버에 전송 등

def main(args=None):
    rclpy.init(args=args)

    converter_node = None
    try:
        converter_node = PointCloudConverterNode()
        rclpy.spin(converter_node)
    except KeyboardInterrupt:
        if converter_node is not None:
            converter_node.get_logger().info("Ctrl+C로 노드 종료")
    finally:
        if converter_node is not None:
            converter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
