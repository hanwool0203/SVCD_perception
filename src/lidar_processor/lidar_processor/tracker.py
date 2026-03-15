#!/usr/bin/env python3
from matplotlib.pyplot import box
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import zmq
import math
import time
from lidar_processor import kalman_tracker as kt

class PointCloudConverterNode(Node):
    def __init__(self):
        super().__init__('pc2np_opt_node')
        self.frame_count = 0 
        self.zaxis_calib = 1.13

        # --- [ZMQ 설정] ---
        # ROS 노드가 '클라이언트(REQ)' 역할입니다.
        # 서버(Conda)가 켜져 있어야만 통신이 성공합니다.
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.REQ) # REQ: 요청하고 응답을 기다림
        self.socket.connect("tcp://localhost:5555") 
        self.get_logger().info("ZMQ 클라이언트 시작: tcp://localhost:5555 연결 시도...")

        # --- [2. Init] 트래커 생성 ---
        # max_frames_to_skip=5: 5프레임(0.5초) 동안은 놓쳐도 ID 유지 (깜빡임 방지)
        # distance_threshold=2.5: 2.5m 이상 움직이면 다른 물체로 간주
        self.tracker = kt.SortTracker(max_frames_to_skip=5, distance_threshold=2.5)

        # '/velodyne_points' 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        
        self.get_logger().info("LiDAR Tracking Node Ready!")

    def lidar_callback(self, msg: PointCloud2):
        """
        PointCloud2 -> NumPy -> ZMQ 전송 -> 결과 수신
        """
        t_start = time.perf_counter() # 타이머 시작

        try:
            # 1. 메시지의 데이터 타입(dtype) 정의
            # VLP-16은 보통 x, y, z, intensity, ring, time 순서로 들어옵니다.
            # 하지만 안전하게 fields 정보를 보고 구조를 만듭니다.
            
            # 간단한 매핑 (ROS 타입 -> NumPy 타입)
            # (필요시 더 많은 타입 추가 가능, 여기선 float32/uint8 위주)
            dtype_list = []
            for field in msg.fields:
                if field.datatype == PointField.FLOAT32:
                    dt = np.float32
                elif field.datatype == PointField.UINT8: # Intensity나 Ring이 uint8일 경우
                    dt = np.uint8
                elif field.datatype == PointField.UINT16:
                    dt = np.uint16
                elif field.datatype == PointField.UINT32:
                    dt = np.uint32
                else:
                    dt = np.float32 # 기본값

                dtype_list.append((field.name, dt))

            # 2. 버퍼에서 바로 NumPy 배열 생성 (여기가 핵심! 속도 100배 향상)
            # msg.data는 bytes 덩어리입니다. 이걸 구조화된 배열로 해석합니다.
            cloud_arr = np.frombuffer(msg.data, dtype=dtype_list)

            # 3. 필요한 (x, y, z, intensity)만 뽑아서 정규화된 float32 배열 만들기
            # (N, ) 구조화 배열 -> (N, 4) 일반 float 배열로 변환
            
            # 미리 빈 배열 생성 (N, 4)
            points_xyzi = np.zeros((cloud_arr.shape[0], 4), dtype=np.float32)
            
            points_xyzi[:, 0] = cloud_arr['x']
            points_xyzi[:, 1] = cloud_arr['y']
            points_xyzi[:, 2] = cloud_arr['z']
            
            # intensity 처리 (이름이 'intensity' 또는 'i' 인지 확인)
            if 'intensity' in cloud_arr.dtype.names:
                points_xyzi[:, 3] = cloud_arr['intensity'].astype(np.float32)
            elif 'i' in cloud_arr.dtype.names:
                 points_xyzi[:, 3] = cloud_arr['i'].astype(np.float32)
            
            # 4. NaN 제거 (OpenPCDet 오류 방지)
            # 어떤 라이다 드라이버는 유효하지 않은 점을 NaN으로 채웁니다.
            points_xyzi = points_xyzi[~np.isnan(points_xyzi).any(axis=1)]

            # 5. 디버깅 로그
            if self.frame_count % 10 == 0: # 1초에 한번만 로그
                self.get_logger().info(
                    f"[Frame {self.frame_count}] 변환 완료: {points_xyzi.shape}, Type: {points_xyzi.dtype}"
                )
                # 데이터 샘플 하나 찍어보기 (잘 들어왔나 확인)
                if len(points_xyzi) > 0:
                     self.get_logger().info(f"Sample[0]: {points_xyzi[0]}")
            
            # =========================================================
            # [처방 적용] 모델이 좋아하는 데이터로 성형수술 
            # =========================================================
            
            # [처방 1] Intensity 스케일 뻥튀기 (0~1 -> 0~255)
            # 최댓값이 1.5 이하인 경우, 스케일이 작다고 판단하고 255를 곱함
            if np.max(points_xyzi[:, 3]) <= 1.5:
                 points_xyzi[:, 3] *= 255.0

            t_preprocess = time.perf_counter() # [측정 중간 1] 전처리 끝난 시간

            # [처방 2] Z축 높이 내리기 (RC카 -> 승용차 높이 흉내)
            # RC카(0.6m) -> nuScenes(1.73m) 차이만큼 내림.
            # 땅속으로 1.5m 정도 내려야 모델이 "아, 바닥이 여기구나" 하고 인식함
            points_xyzi[:, 2] -= self.zaxis_calib

            # =========================================================

            # ---------------------------------------------------------
            # 2. ZMQ로 Conda 서버에 전송 (REQ)
            # ---------------------------------------------------------
            # 데이터 형태 정보를 먼저 보냅니다 (JSON)
            meta_data = {
                'dtype': str(points_xyzi.dtype),
                'shape': points_xyzi.shape
            }
            
            # send_json(flags=SNDMORE) -> 그 다음에 send(bytes)
            self.socket.send_json(meta_data, flags=zmq.SNDMORE)
            self.socket.send(points_xyzi.tobytes())

            # ---------------------------------------------------------
            # 3. 결과 수신 대기 (REP를 기다림 - Blocking)
            # ---------------------------------------------------------
            # 주의: 서버가 꺼져있으면 여기서 코드가 멈춥니다(Hang).
            # 서버가 응답을 줘야만 다음 줄로 넘어갑니다.
            
            res_meta = self.socket.recv_json()
            res_bytes = self.socket.recv()

            # 바이트를 다시 NumPy 배열(Bounding Boxes)로 복원
            # 예: [[x, y, z, w, l, h, yaw], ...]
            pred_boxes = np.frombuffer(res_bytes, dtype=res_meta['dtype'])
            pred_boxes = pred_boxes.reshape(res_meta['shape'])

            # [측정 중간 2] 추론(통신) 끝난 시간
            t_inference = time.perf_counter()

            # =========================================================
            # [3. Tracking] 핵심 이식 파트
            # =========================================================
            tracked_objects = np.empty((0, 11))
            
            if len(pred_boxes) > 0:
                # Detection(N, 11) -> Tracker -> Tracked Objects(N, 11)
                # 이제 각 박스에 고유 ID가 부여되고, 튀는 값이 보정됩니다.
                tracked_objects = self.tracker.update(pred_boxes)
            
            # 시각화에는 'tracked_objects'를 넘겨줍니다.
            if len(tracked_objects) > 0:
                self.publish_markers(tracked_objects, msg.header)

            # [측정 종료] 모든 작업 완료 시간
            t_end = time.perf_counter()

            # ---------------------------------------------------------
            # 4. 레이턴시 계산 및 출력 (단위: ms)
            # ---------------------------------------------------------
            preprocess_ms = (t_preprocess - t_start) * 1000
            inference_ms = (t_inference - t_preprocess) * 1000
            viz_ms = (t_end - t_inference) * 1000
            total_ms = (t_end - t_start) * 1000
                
            # 10프레임마다 로그 출력 (너무 빠르면 정신없으므로)
            if self.frame_count % 10 == 0:
                self.get_logger().info(f"=========== [Frame {self.frame_count}] ===========")
                self.get_logger().info(f"감지된 객체: {len(pred_boxes)}개")
                self.get_logger().info(f"전처리 시간: {preprocess_ms:.2f} ms")
                self.get_logger().info(f"추론(ZMQ) 시간: {inference_ms:.2f} ms")
                self.get_logger().info(f"시각화 시간: {viz_ms:.2f} ms")
                self.get_logger().info(f"👉 총 레이턴시: {total_ms:.2f} ms (Target: < 100ms)")
                self.get_logger().info(f"=====================================")

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"에러 발생: {e}")

    def publish_markers(self, boxes, header):
            marker_array = MarkerArray()
            
            # 잔상 제거 (필수)
            del_marker = Marker()
            del_marker.action = Marker.DELETEALL
            marker_array.markers.append(del_marker)

            # -----------------------------------------------------------
            # [nuScenes 10 Class Color Mapping]
            # -----------------------------------------------------------
            CLASS_COLORS = {
                1: (0.0, 1.0, 0.0),    # Car (Green)
                2: (0.0, 0.5, 1.0),    # Truck (Blue)
                3: (0.5, 0.5, 0.5),    # Construction Vehicle (Grey)
                4: (0.0, 0.0, 1.0),    # Bus (Dark Blue)
                5: (0.5, 0.0, 0.5),    # Trailer (Purple)
                6: (0.8, 0.8, 0.0),    # Barrier (Dark Yellow)
                7: (0.0, 1.0, 1.0),    # Motorcycle (Cyan)
                8: (0.0, 0.7, 0.3),    # Bicycle (Teal)
                9: (1.0, 0.0, 0.0),    # Pedestrian (Red)
                10: (1.0, 0.5, 0.0),   # Traffic Cone (Orange)
            }
            DEFAULT_COLOR = (1.0, 1.0, 1.0) # Unknown (White)
            
            # 라벨 텍스트 매핑
            LABEL_MAP = {1:'Car', 2:'Trk', 3:'Const', 4:'Bus', 5:'Trl', 
                        6:'Bar', 7:'Moto', 8:'Bike', 9:'Ped', 10:'Cone'}
            # -----------------------------------------------------------

            for i, box in enumerate(boxes):
                # 데이터 파싱 (안전 장치 추가)
                # 기본 OpenPCDet-nuScenes 출력: [x, y, z, dx, dy, dz, heading, vx, vy, score, label] (총 11개)
                
                # 1. Label 파싱 (마지막 값)
                try:
                    obj_id = int(box[7])     # ID는 7번 인덱스
                    class_id = int(box[10])  # Label은 맨 뒤 (10번)
                except:
                    obj_id = i
                    class_id = 0
                
                # 2. Score 파싱 (뒤에서 두 번째 값, 인덱스 9)
                score = 0.0
                if box.shape[0] >= 11:
                    score = float(box[9])

                # -------------------------------------------------------
                # [Marker 1] 객체 박스 (Cube)
                # -------------------------------------------------------
                marker = Marker()
                marker.header = header
                marker.ns = "objects"
                marker.id = obj_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # 좌표 및 크기
                marker.pose.position.x = float(box[0])
                marker.pose.position.y = float(box[1])
                marker.pose.position.z = float(box[2]) + self.zaxis_calib 

                yaw = float(box[6])
                marker.pose.orientation.z = math.sin(yaw / 2.0)
                marker.pose.orientation.w = math.cos(yaw / 2.0)
                
                marker.scale.x = float(box[3])
                marker.scale.y = float(box[4])
                marker.scale.z = float(box[5])

                # 색상 적용
                r, g, b = CLASS_COLORS.get(class_id, DEFAULT_COLOR)
                marker.color.r, marker.color.g, marker.color.b = r, g, b
                marker.color.a = 0.6
                marker.lifetime.nanosec = 200000000 # 0.2초
                
                marker_array.markers.append(marker)
                
                # -------------------------------------------------------
                # [Marker 2] 텍스트 정보 (Class + Score)
                # -------------------------------------------------------
                text_marker = Marker()
                text_marker.header = header
                text_marker.ns = "info"
                text_marker.id = i + 10000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose.position.x = marker.pose.position.x
                text_marker.pose.position.y = marker.pose.position.y
                text_marker.pose.position.z = marker.pose.position.z + (marker.scale.z / 2.0) + 0.5
                
                text_marker.scale.z = 0.4 # 글자 크기
                text_marker.color.r, text_marker.color.g, text_marker.color.b = 1.0, 1.0, 1.0
                text_marker.color.a = 1.0

                label_str = LABEL_MAP.get(class_id, f"C{class_id}")
                
                # "Car-12" 형식으로 표시 (ID 확인용)
                text_marker.text = f"{label_str}-{obj_id}"
                
                text_marker.lifetime.nanosec = 200000000
                marker_array.markers.append(text_marker)

            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
