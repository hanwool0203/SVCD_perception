#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from perception_interfaces.msg import ObjectArray, ObjectInfo
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
import zmq
import math
import time

class LidarOdomFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_odom_fusion_node')
        
        # --- [핵심 설정] ---
        self.MAX_SWEEPS = 5  # 속도 추론을 위해 10프레임 필수!
        self.sweep_buffer = deque(maxlen=self.MAX_SWEEPS)
        self.Z_OFFSET = 1.13
        self.frame_count = 0

        # [시각화 최적화] Rviz 딜레이의 주범을 잡기 위한 설정
        self.VISUALIZE_STACKED = False # Rviz에 점을 뿌릴지 말지
        self.VIS_DOWNSAMPLE = 5       # 5개 중 1개만 보냄 (데이터 80% 절약)
        self.VIS_SKIP = 2             # 2번에 1번만 보냄 (부하 절반 감소)
        self.vis_counter = 0
        
        # --- [ZMQ 설정] ---
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:5555")
        self.get_logger().info("ZMQ 클라이언트 연결 (속도 추론 모드)")

        # --- [구독] ---
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/velodyne_points', qos_profile=10)
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/kiss/odometry', qos_profile=10)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.odom_sub], queue_size=20, slop=0.5
        )
        self.ts.registerCallback(self.callback)

        # --- [발행] ---
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.stacked_pub = self.create_publisher(PointCloud2, '/stacked_points', 10) # 디버깅용
        self.objects_pub = self.create_publisher(ObjectArray, '/planning/objects', 10)

    def get_matrix_from_odom(self, odom_msg):
        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        trans = np.array([p.x, p.y, p.z])
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = trans
        return T

    def pc2_to_numpy(self, msg):
        # 안전 파싱
        dtype_list = []
        for field in msg.fields:
            if field.datatype == 7: dt = np.float32
            elif field.datatype == 2: dt = np.uint8
            elif field.datatype == 4: dt = np.uint16
            else: dt = np.float32
            dtype_list.append((field.name, dt))
            
        cloud_arr = np.frombuffer(msg.data, dtype=dtype_list)
        points = np.zeros((cloud_arr.shape[0], 4), dtype=np.float32)
        points[:, 0] = cloud_arr['x']
        points[:, 1] = cloud_arr['y']
        points[:, 2] = cloud_arr['z']
        
        if 'intensity' in cloud_arr.dtype.names:
            points[:, 3] = cloud_arr['intensity'].astype(np.float32)
        elif 'i' in cloud_arr.dtype.names:
            points[:, 3] = cloud_arr['i'].astype(np.float32)
            
        return points[~np.isnan(points).any(axis=1)]
    
    def numpy_to_pc2(self, points, header):
        # [복구됨] Rviz 시각화를 위해 Numpy -> PC2 변환 함수가 필요합니다!
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = True
        msg.point_step = 20
        msg.row_step = 20 * points.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1),
            PointField(name='intensity', offset=12, datatype=7, count=1),
            PointField(name='timestamp', offset=16, datatype=7, count=1),
        ]
        msg.data = points.astype(np.float32).tobytes()
        return msg
    
    def callback(self, lidar_msg, odom_msg):

        t_start = time.perf_counter()
        # self.get_logger().info(f"📥 현재 버퍼 크기: {len(self.sweep_buffer)} / {self.MAX_SWEEPS}")

        try:
            # 1. 데이터 변환
            points_np = self.pc2_to_numpy(lidar_msg)
            
            # [처방 1] Intensity 보정 (여기서 미리 함)
            if np.max(points_np[:, 3]) <= 1.5:
                points_np[:, 3] *= 255.0
            
            current_pose = self.get_matrix_from_odom(odom_msg)
            # Bag 파일 재생 시에는 메시지 헤더 시간을 써야 함!
            current_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
            
            self.sweep_buffer.append({
                'points': points_np,
                'pose': current_pose,
                'time': current_time
            })
            
            # 2. 스택킹 (Stacking)
            all_points_list = []
            inv_current_pose = np.linalg.inv(current_pose)

            total_timelag = 0.0 # 디버깅용
            
            for sweep in self.sweep_buffer:
                past_points = sweep['points']
                past_pose = sweep['pose']
                
                # (A) 좌표 변환 (과거 -> 현재)
                xyz = past_points[:, :3]
                ones = np.ones((xyz.shape[0], 1))
                xyz_homo = np.hstack([xyz, ones])
                
                rel_transform = np.matmul(inv_current_pose, past_pose)
                transformed_xyz = np.matmul(rel_transform, xyz_homo.T).T[:, :3]
                
                # (B) Time Lag 계산 (중요: 0 ~ 0.5 사이 양수여야 함)
                raw_time_lag = current_time - sweep['time']
                # self.get_logger().info(f"TimeLag: {raw_time_lag:.4f}")

                total_timelag += raw_time_lag

                # [해결책 1] 음수 방지 & 단위 확인
                # 만약 값이 1000 이상이면 단위를 잘못 쓴 것 (나노초 이슈)
                if raw_time_lag > 100.0: 
                    # 혹시 나노초 단위로 들어왔다면 초 단위로 변환 (예시)
                    # raw_time_lag *= 1e-9 
                    pass # 일단 로그 보고 판단

                # [해결책 2] 범위 강제 제한 (Clamping)
                # nuScenes는 0.5초까지만 배웠으므로, 0.5초 넘는 건 그냥 0.5초라고 거짓말 치거나
                # 아예 0.5초 이내인 데이터만 스택킹해야 함.
                # 일단 0.5를 넘지 않게 막아봅시다.
                time_lag = max(0.0, min(0.5, raw_time_lag))
                
                # [안전장치] 시간이 꼬여서 음수거나 너무 크면 0으로 보정 (모델 보호)
                if time_lag < 0 or time_lag > 1.0:
                    time_lag = 0.0
                # [Time Lag 강제 0 테스트: 주석 풀면 속도 추론은 포기하지만 Detection 성능 확인 가능]
                # time_lag = 0.0

                time_col = np.full((transformed_xyz.shape[0], 1), time_lag)

                # [x, y, z, i, t]
                points_5d = np.hstack([transformed_xyz, past_points[:, 3:4], time_col])
                all_points_list.append(points_5d)
            
            if not all_points_list: return

            stacked_cloud_np = np.vstack(all_points_list).astype(np.float32)

            # self.get_logger().info(f"평균 TimeLag: {total_timelag / len(self.sweep_buffer):.4f}")

            # [처방 2] Z축 높이 보정 (Stacking 다 하고 나서 일괄 적용)
            stacked_cloud_np[:, 2] -= self.Z_OFFSET

            # [중간 시간 1] 전처리(스택킹) 완료
            t_preprocess = time.perf_counter()

            # 3. ZMQ 전송
            meta_data = {'dtype': str(stacked_cloud_np.dtype), 'shape': stacked_cloud_np.shape}
            self.socket.send_json(meta_data, flags=zmq.SNDMORE)
            self.socket.send(stacked_cloud_np.tobytes())

            # 4. 결과 수신
            res_meta = self.socket.recv_json()
            res_bytes = self.socket.recv()
            pred_boxes = np.frombuffer(res_bytes, dtype=res_meta['dtype']).reshape(res_meta['shape'])

            t_inference = time.perf_counter()

            if len(pred_boxes) > 0:
                self.publish_markers(pred_boxes, lidar_msg.header)

            # 5. 시각화용 발행 (최적화 적용)
            if self.VISUALIZE_STACKED:
                self.vis_counter += 1
                if self.vis_counter % self.VIS_SKIP == 0:
                    # 시각화용 복사본 생성 (모델용 데이터 훼손 방지)
                    vis_points = stacked_cloud_np[::self.VIS_DOWNSAMPLE].copy()
                    vis_points[:, 2] += self.Z_OFFSET 
                    out_msg = self.numpy_to_pc2(vis_points, lidar_msg.header)
                    self.stacked_pub.publish(out_msg)
            
            # [종료 시간]
            t_end = time.perf_counter() 

            # ---------------------------------------------------------
            # 레이턴시 출력
            # ---------------------------------------------------------
            self.frame_count += 1
            
            if self.frame_count % 2 == 0 :
                stack_ms = (t_preprocess - t_start) * 1000
                inf_ms = (t_inference - t_preprocess) * 1000
                vis_ms = (t_end - t_inference) * 1000
                total_ms = (t_end - t_start) * 1000
                
                self.get_logger().info(f"== [Frame Buffer: {len(self.sweep_buffer)}] ==")
                self.get_logger().info(f"🛠️ 스택킹 연산: {stack_ms:.2f} ms")
                self.get_logger().info(f"🧠 추론(ZMQ): {inf_ms:.2f} ms")
                self.get_logger().info(f"📺 시각화: {vis_ms:.2f} ms")
                self.get_logger().info(f"⏱️ 총 레이턴시: {total_ms:.2f} ms")
                self.get_logger().info(f"=============================")

        except Exception as e:
            self.get_logger().error(f"Fusion Error: {e}")

    def publish_markers(self, boxes, header):
        marker_array = MarkerArray()
        # 잔상 제거
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        # -----------------------------------------------------------
        # [nuScenes 10 Class Color Mapping]
        # 순서: car, truck, construction_vehicle, bus, trailer, 
        #       barrier, motorcycle, bicycle, pedestrian, traffic_cone
        # -----------------------------------------------------------
        CLASS_COLORS = {
            1: (0.0, 1.0, 0.0),    # 1. Car (Green - 초록색)
            2: (0.0, 0.5, 1.0),    # 2. Truck (Blue - 파란색)
            3: (0.5, 0.5, 0.5),    # 3. Construction Vehicle (Grey - 회색)
            4: (0.0, 0.0, 1.0),    # 4. Bus (Dark Blue - 진한 파랑)
            5: (0.5, 0.0, 0.5),    # 5. Trailer (Purple - 보라색)
            6: (0.8, 0.8, 0.0),    # 6. Barrier (Dark Yellow - 어두운 노랑)
            7: (0.0, 1.0, 1.0),    # 7. Motorcycle (Cyan - 청록색)
            8: (0.0, 0.7, 0.3),    # 8. Bicycle (Teal - 짙은 녹색)
            9: (1.0, 0.0, 0.0),    # 9. Pedestrian (Red - 빨간색)
            10: (1.0, 0.5, 0.0),   # 10. Traffic Cone (Orange - 주황색)
        }
        DEFAULT_COLOR = (1.0, 1.0, 1.0) # 알 수 없음 (White)
        # -----------------------------------------------------------

        for i, box in enumerate(boxes):
            marker = Marker()
            marker.header = header
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # [처방 2 원복] 시각화할 땐 다시 위로 올림
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2]) + self.Z_OFFSET 

            yaw = float(box[6])
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x = float(box[3])
            marker.scale.y = float(box[4])
            marker.scale.z = float(box[5])

            # [색상 적용] box의 마지막 값(label)을 읽어옴
            try:
                class_id = int(box[-1]) 
            except:
                class_id = 0

            r, g, b = CLASS_COLORS.get(class_id, DEFAULT_COLOR)

            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            marker.color.a = 0.6
            marker.lifetime.nanosec = 200000000
            marker_array.markers.append(marker)
            
            # [텍스트 정보]
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "info"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = marker.pose.position.z + 1.0
            text_marker.scale.z = 0.5
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0

            # 속도 정보 확인 (box 길이가 9보다 크면 속도가 있다고 가정)
            velocity_text = ""
            if box.shape[0] > 9: 
                 vx, vy = float(box[7]), float(box[8]) # 위치 주의 (Label이 맨 뒤라면 속도는 그 앞일 수 있음)
                 # 보통: x,y,z,dx,dy,dz,h, vx, vy, label (10개) 
                 # 혹은: x,y,z,dx,dy,dz,h, score, label (9개) -> 이 경우 속도 없음
                 
                 # 만약 box[7], box[8]이 속도가 아니라 score, label이라면 값이 이상하게 찍힐 수 있습니다.
                 # 일단 속도가 있다고 가정하고 계산:
                 v_mag = math.sqrt(vx**2 + vy**2)
                 # 값이 너무 크거나 이상하면 속도가 아닐 확률 높음 (nuScenes max speed check)
                 if v_mag < 100: 
                    velocity_text = f"\n{v_mag:.1f}m/s"

            # 텍스트에 클래스 이름 약어 표시
            label_map = {1:'Car', 2:'Trk', 3:'Const', 4:'Bus', 5:'Trl', 
                         6:'Bar', 7:'Moto', 8:'Bike', 9:'Ped', 10:'Cone'}
            label_str = label_map.get(class_id, f"{class_id}")
            
            text_marker.text = f"{label_str}{velocity_text}"
            
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarOdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
