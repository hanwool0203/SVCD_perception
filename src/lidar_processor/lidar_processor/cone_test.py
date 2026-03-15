#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
import zmq
import math
import time
from lidar_processor import kalman_tracker as kt

class LidarOdomFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_odom_fusion_node')
        
        # --- [핵심 설정] ---
        self.MAX_SWEEPS = 5  # 속도 추론을 위해 10프레임 필수!
        self.sweep_buffer = deque(maxlen=self.MAX_SWEEPS)
        self.Z_OFFSET = 1.13

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
        self.lane_pub = self.create_publisher(Marker, '/detected_lanes', 10)
        self.stacked_pub = self.create_publisher(PointCloud2, '/stacked_points', 10) # 디버깅용

        self.tracker = kt.SortTracker(max_frames_to_skip=15, distance_threshold=3.5)

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
    
# [헬퍼 함수] 거리 계산
    def dist_sq(self, p1, p2):
        return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

    # [헬퍼 함수] 부채꼴 영역 내 포함 여부 확인
    def is_in_sector(self, x, y, is_left):
        # 거리 체크 (차량 기준 5m 이내의 가까운 콘만 시작점으로 인정)
        dist = math.sqrt(x**2 + y**2)
        if dist > 5.0 or dist < 0.5: # 너무 먼 것은 시작점 X, 너무 가까운(센서 노이즈) 것도 제외
            return False
            
        # 각도 체크 (atan2 결과는 -pi ~ pi)
        angle = math.atan2(y, x) # 라디안
        
        if is_in_sector_debug := False: # 디버깅 필요시 True
            print(f"Angle: {math.degrees(angle):.2f}, Dist: {dist:.2f}, Left?: {is_left}")

        # 왼쪽 부채꼴: 0도 ~ 90도 (정면 ~ 좌측)
        if is_left:
            return 0.0 < angle < 1.57 # 약 0 ~ 90도
        # 오른쪽 부채꼴: -90도 ~ 0도 (우측 ~ 정면)
        else:
            return -1.57 < angle < 0.0 # 약 -90 ~ 0도

    def get_line_chain(self, start_idx, all_cones, max_gap=1.0):
        """
        start_idx: 시작 콘의 인덱스
        all_cones: 전체 콘 리스트 [(x, y), ...]
        max_gap: 연결할 최대 거리 (1.0m)
        """
        line_indices = [start_idx]
        visited = set([start_idx])
        current_idx = start_idx
        
        while True:
            best_next_idx = -1
            min_dist_sq = max_gap**2 + 0.01 # 기준치보다 조금 크게 초기화
            
            curr_x, curr_y = all_cones[current_idx]
            
            # 남은 콘들 중에서 가장 가까운 놈 찾기 (Greedy)
            for i, (cx, cy) in enumerate(all_cones):
                if i in visited: continue
                
                # 거리 계산
                d_sq = (curr_x - cx)**2 + (curr_y - cy)**2
                
                # [조건 1] 지정된 반경(1m) 이내여야 함
                if d_sq <= max_gap**2:
                    # [조건 2] 차량 뒤쪽으로 역주행하는 연결 방지 (옵션)
                    # 콘이 차량보다 멀어지는 방향(x 증가)인 경우를 우선할 수 있음
                    # 여기서는 단순히 거리 가장 가까운 것을 선택
                    if d_sq < min_dist_sq:
                        min_dist_sq = d_sq
                        best_next_idx = i
            
            # 갈 곳이 있으면 이동
            if best_next_idx != -1:
                line_indices.append(best_next_idx)
                visited.add(best_next_idx)
                current_idx = best_next_idx
            else:
                # 반경 내에 연결할 콘이 없으면 종료
                break
                
        return line_indices

    def draw_lanes(self, boxes, header):
        """
        부채꼴 시작점 탐색 -> 반경 1m 이내 근접 연결 (Greedy Chaining)
        """
        # 1. 유효한 콘 데이터만 추출 (좌표 리스트화)
        cones = [] # [(x, y), ...]
        
        for box in boxes:
            x, y = float(box[0]), float(box[1])
            dx, dy = float(box[3]), float(box[4])
            
            # [필터링]
            if dx > 1.2 or dy > 1.2: continue # 너무 큰 차는 제외
            if x < -1.0: continue # 차 뒤에 있는 건 제외
            
            cones.append((x, y))

        if not cones: return

        # 2. 시작점(Anchor) 찾기 - 부채꼴 영역 내에서 가장 가까운 콘
        left_start_idx = -1
        right_start_idx = -1
        min_l_dist = 999.0
        min_r_dist = 999.0

        for i, (cx, cy) in enumerate(cones):
            dist = math.sqrt(cx**2 + cy**2)
            
            # 왼쪽 후보 찾기
            if self.is_in_sector(cx, cy, is_left=True):
                if dist < min_l_dist:
                    min_l_dist = dist
                    left_start_idx = i
            
            # 오른쪽 후보 찾기
            if self.is_in_sector(cx, cy, is_left=False):
                if dist < min_r_dist:
                    min_r_dist = dist
                    right_start_idx = i

        # 3. 체이닝 (선을 잇는 과정)
        # 왼쪽 라인
        left_line_pts = []
        if left_start_idx != -1:
            indices = self.get_line_chain(left_start_idx, cones, max_gap=3.0) # 1m 너무 빡빡하면 1.5m로 완화
            left_line_pts = [cones[i] for i in indices]

        # 오른쪽 라인
        right_line_pts = []
        if right_start_idx != -1:
            # 왼쪽 라인에 쓰인 점이 오른쪽 시작점이라면 중복 방지 (드문 경우)
            if right_start_idx != left_start_idx: 
                indices = self.get_line_chain(right_start_idx, cones, max_gap=3.0)
                right_line_pts = [cones[i] for i in indices]

        # 4. 시각화 마커 생성 (LINE_STRIP 사용 - 점들을 순서대로 이음)
        lane_marker = Marker()
        lane_marker.header = header
        lane_marker.ns = "lanes_chain"
        lane_marker.id = 0
        lane_marker.type = Marker.LINE_LIST # A-B, C-D... 가 아니라 A-B, B-C 로 그리려면 LINE_STRIP이 좋으나, 구현상 LIST로 끊어 그림
        lane_marker.action = Marker.ADD
        lane_marker.scale.x = 0.15
        lane_marker.color.a = 1.0
        lane_marker.color.r = 1.0; lane_marker.color.g = 1.0; lane_marker.color.b = 0.0 # Yellow

        # 왼쪽 라인 마커 채우기
        if len(left_line_pts) >= 2:
            for i in range(len(left_line_pts) - 1):
                p1 = Point(x=left_line_pts[i][0], y=left_line_pts[i][1], z=-1.0)
                p2 = Point(x=left_line_pts[i+1][0], y=left_line_pts[i+1][1], z=-1.0)
                lane_marker.points.append(p1)
                lane_marker.points.append(p2)
        
        # 오른쪽 라인 마커 채우기
        if len(right_line_pts) >= 2:
            for i in range(len(right_line_pts) - 1):
                p1 = Point(x=right_line_pts[i][0], y=right_line_pts[i][1], z=-1.0)
                p2 = Point(x=right_line_pts[i+1][0], y=right_line_pts[i+1][1], z=-1.0)
                lane_marker.points.append(p1)
                lane_marker.points.append(p2)

        self.lane_pub.publish(lane_marker)

    def callback(self, lidar_msg, odom_msg):

        t_start = time.perf_counter()
        self.get_logger().info(f"📥 현재 버퍼 크기: {len(self.sweep_buffer)} / {self.MAX_SWEEPS}")

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
                self.get_logger().info(f"TimeLag: {raw_time_lag:.4f}")

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

            self.get_logger().info(f"평균 TimeLag: {total_timelag / len(self.sweep_buffer):.4f}")

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

            # 원래 모델이 추론하는 결과를 rviz로 쏘는 코드
            # if len(pred_boxes) > 0:
            #     self.publish_markers(pred_boxes, lidar_msg.header)

            tracked_boxes = self.tracker.update(pred_boxes)

            if len(tracked_boxes) > 0:
                self.publish_markers(tracked_boxes, lidar_msg.header)
                self.draw_lanes(tracked_boxes, lidar_msg.header)

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

        for i, box in enumerate(boxes):

            obj_id = int(box[7])

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

            # ID별 고유 색상
            np.random.seed(obj_id)
            marker.color.r = np.random.rand()
            marker.color.g = np.random.rand()
            marker.color.b = np.random.rand()
            marker.color.a = 0.6
            marker.lifetime.nanosec = 200000000 # 0.2s
            marker_array.markers.append(marker)

            # 텍스트 정보 표시
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "info"
            text_marker.id = obj_id + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = marker.pose.position.z + 1.2
            text_marker.scale.z = 0.5
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
            
            # 속도 정보가 있으면 표시
            if box.shape[0] >= 10:
                vx, vy = float(box[8]), float(box[9])
                velocity = math.sqrt(vx**2 + vy**2)
                text_marker.text = f"ID:{obj_id}\n{velocity:.1f}m/s"
            else:
                text_marker.text = f"ID:{obj_id}"
                
            marker_array.markers.append(text_marker)

            # marker.color.a = 0.6
            # marker.color.g = 1.0
            # marker.lifetime.nanosec = 200000000
            # marker_array.markers.append(marker)
            
            # # [속도 시각화] 모델이 속도(vx, vy)를 줬는지 확인 (인덱스 7, 8)
            # if box.shape[0] >= 9:
            #     vx, vy = float(box[7]), float(box[8])
            #     velocity = math.sqrt(vx**2 + vy**2)
                
            #     # 속도 텍스트 띄우기
            #     text_marker = Marker()
            #     text_marker.header = header
            #     text_marker.ns = "velocity"
            #     text_marker.id = i + 1000
            #     text_marker.type = Marker.TEXT_VIEW_FACING
            #     text_marker.action = Marker.ADD
            #     text_marker.pose.position.x = marker.pose.position.x
            #     text_marker.pose.position.y = marker.pose.position.y
            #     text_marker.pose.position.z = marker.pose.position.z + 1.0
            #     text_marker.scale.z = 0.5
            #     text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
            #     text_marker.text = f"Vel: {velocity:.1f} m/s"
            #     marker_array.markers.append(text_marker)

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