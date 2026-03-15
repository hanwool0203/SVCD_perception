#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment

# =============================================================================
# 1. 칼만 필터 기반 개별 객체 트래커 (BoxTracker)
# =============================================================================
class KalmanBoxTracker(object):
    count = 0
    def __init__(self, bbox):
        # 상태 벡터: [u, v, s, r, u_dot, v_dot, s_dot]
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],  
                              [0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
        self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

        self.kf.R[2:,2:] *= 10.
        self.kf.P[4:,4:] *= 1000.
        self.kf.P *= 10.
        self.kf.Q[-1,-1] *= 0.01
        self.kf.Q[4:,4:] *= 0.01

        self.kf.x[:4] = self.convert_bbox_to_z(bbox)
        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.history = []
        self.hits = 0
        self.hit_streak = 0
        self.age = 0
        self.original_props = bbox[4:] 

    def update(self, bbox):
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(self.convert_bbox_to_z(bbox))
        self.original_props = bbox[4:] 

    def predict(self):
        if((self.kf.x[6]+self.kf.x[2])<=0):
            self.kf.x[6] *= 0.0
        self.kf.predict()
        self.age += 1
        if(self.time_since_update>0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(self.convert_bbox_to_z(self.kf.x))
        return self.history[-1]

    def get_state(self):
        return self.convert_x_to_bbox(self.kf.x)

    def convert_bbox_to_z(self, bbox):
        # [Critical Fix] 모든 입력을 강제로 float 스칼라로 변환
        # bbox가 (N, 1) 형태의 numpy array일 수도 있고 list일 수도 있음
        # float()로 감싸서 확실하게 숫자 하나로 만듦
        
        try:
            w = float(bbox[4]) if len(bbox) > 4 else 1.0
            h = float(bbox[5]) if len(bbox) > 5 else 1.0
            x = float(bbox[0])
            y = float(bbox[1])
        except:
            # 혹시라도 변환 실패 시 기본값
            w, h, x, y = 1.0, 1.0, 0.0, 0.0
        
        # 0으로 나누기 방지
        if h < 1e-3: h = 0.1
        if w < 1e-3: w = 0.1

        s = w * h
        r = w / h
        
        # dtype을 명시적으로 float32로 지정하여 Object Array 생성 방지
        return np.array([x, y, s, r], dtype=np.float32).reshape((4, 1))

    def convert_x_to_bbox(self, x, score=None):
        # [Critical Fix] 여기도 마찬가지로 float 강제 형변환
        try:
            cx = float(x[0])
            cy = float(x[1])
            s = float(x[2]) if float(x[2]) > 0 else 0.1
            r = float(x[3]) if float(x[3]) > 0 else 1.0
        except:
             cx, cy, s, r = 0.0, 0.0, 0.1, 1.0

        w = np.sqrt(s * r)
        h = s / w if w > 1e-3 else 0.1
        
        return np.array([cx, cy, 0, 0, w, h, 0], dtype=np.float32).reshape((1, 7))

# =============================================================================
# 2. 전체 트래커 관리자 (SORT)
# =============================================================================
class SortTracker(object):
    def __init__(self, max_age=5, min_hits=1, iou_threshold=3.0):
        self.max_age = max_age
        self.min_hits = min_hits
        self.distance_threshold = iou_threshold 
        self.trackers = []
        self.frame_count = 0

    def update(self, dets=np.empty((0, 7))):
        self.frame_count += 1
        
        trks = np.zeros((len(self.trackers), 5))
        to_del = []
        
        for t, trk in enumerate(trks):
            # [Safety] 예측 수행
            try:
                pred = self.trackers[t].predict()
                # 예측 결과 shape 확인 (4, 1)이어야 함
                pos = pred.flatten() 
            except Exception:
                to_del.append(t)
                continue

            # [Fix] NaN 체크 (float array이므로 이제 정상 작동)
            if np.any(np.isnan(pos)):
                to_del.append(t)
                continue
            
            # pos: [x, y, s, r]
            trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
        
        # 에러 난 트래커 삭제 (뒤에서부터 삭제해야 인덱스 안 꼬임)
        for t in reversed(to_del):
            if t < len(self.trackers):
                self.trackers.pop(t)

        # 유효한 트래커만 남겨서 매칭 진행
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        
        matched, unmatched_dets, unmatched_trks = self.associate_detections_to_trackers(dets, trks)

        # 매칭된 트래커 업데이트
        for t, trk in enumerate(self.trackers):
            if t not in unmatched_trks:
                d = dets[matched[np.where(matched[:, 1] == t)[0][0]]]
                trk.update(d)

        # 매칭 안 된 디텍션 -> 신규 트래커 생성
        for i in unmatched_dets:
            trk = KalmanBoxTracker(dets[i, :])
            self.trackers.append(trk)

        ret = []
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            try:
                state_vec = trk.get_state()[0]
                # NaN 체크
                if np.any(np.isnan(state_vec)):
                    self.trackers.pop(i-1)
                    i -= 1
                    continue
            except Exception:
                i -= 1
                continue
            
            d = state_vec
            
            # Coasting (잔상 유지) 로직
            if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
                ret.append(np.concatenate((d, [trk.id], trk.original_props)).reshape(1, -1))
            
            elif (trk.time_since_update <= self.max_age) and (trk.hit_streak >= self.min_hits):
                ret.append(np.concatenate((d, [trk.id], trk.original_props)).reshape(1, -1))

            i -= 1
            if(trk.time_since_update > self.max_age):
                self.trackers.pop(i)
                
        if(len(ret)>0): return np.concatenate(ret)
        return np.empty((0,7))

    def associate_detections_to_trackers(self, detections, trackers):
        if(len(trackers)==0): return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
        
        d_mat = np.zeros((len(detections), len(trackers)), dtype=np.float32)
        for d, det in enumerate(detections):
            for t, trk in enumerate(trackers):
                dist = np.sqrt((det[0]-trk[0])**2 + (det[1]-trk[1])**2)
                d_mat[d,t] = dist

        matched_indices = linear_sum_assignment(d_mat)
        matched_indices = np.array(list(zip(matched_indices[0], matched_indices[1])))

        unmatched_detections = []
        for d, det in enumerate(detections):
            if(d not in matched_indices[:,0]):
                unmatched_detections.append(d)
        
        unmatched_trackers = []
        for t, trk in enumerate(trackers):
            if(t not in matched_indices[:,1]):
                unmatched_trackers.append(t)

        matches = []
        for m in matched_indices:
            if(d_mat[m[0], m[1]] > self.distance_threshold):
                unmatched_detections.append(m[0])
                unmatched_trackers.append(m[1])
            else:
                matches.append(m.reshape(1,2))
        
        if(len(matches)==0):
            matches = np.empty((0,2),dtype=int)
        else:
            matches = np.concatenate(matches, axis=0)

        return matches, np.array(unmatched_detections), np.array(unmatched_trackers)

# =============================================================================
# 3. ROS2 노드
# =============================================================================
class MarkerTrackerNode(Node):
    def __init__(self):
        super().__init__('marker_tracker_node')
        
        self.tracker = SortTracker(max_age=10, min_hits=2, iou_threshold=2.5)
        self.sub = self.create_subscription(MarkerArray, '/detected_objects', self.callback, 10)
        self.pub = self.create_publisher(MarkerArray, '/tracked_objects', 10)
        self.get_logger().info("Marker Tracker Started (Robust Mode)")

    def callback(self, msg):
        dets = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL: continue
            
            # 입력 데이터 형변환 및 검증
            x = float(marker.pose.position.x)
            y = float(marker.pose.position.y)
            z = float(marker.pose.position.z)
            dx = float(marker.scale.x)
            dy = float(marker.scale.y)
            dz = float(marker.scale.z)
            
            if dx < 0.01: dx = 0.1
            if dy < 0.01: dy = 0.1
            if dz < 0.01: dz = 0.1

            # 리스트로 저장 (numpy array 아님)
            dets.append([x, y, z, 0.0, dx, dy, dz])
        
        if len(dets) == 0:
            return

        # 여기서 한 번에 깔끔한 float array로 만듦
        dets_np = np.array(dets, dtype=np.float32)
        
        tracked_objects = self.tracker.update(dets_np)

        out_msg = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        out_msg.markers.append(del_marker)

        for i, obj in enumerate(tracked_objects):
            if np.any(np.isnan(obj)): continue

            tx, ty = float(obj[0]), float(obj[1])
            
            # ID 파싱
            try:
                # obj 구조가 [x, y, z, ..., id, props...]
                # 보통 인덱스 7이나 4에 있음
                if len(obj) >= 8:
                    tid = int(obj[7])
                else:
                    tid = int(obj[4])
            except:
                tid = i

            # 크기 정보 파싱
            if len(obj) >= 11:
                tdx, tdy, tdz = float(obj[8]), float(obj[9]), float(obj[10])
            else:
                tdx, tdy, tdz = 0.5, 0.5, 1.0

            marker = Marker()
            marker.header = msg.markers[0].header if msg.markers else msg.header
            marker.ns = "tracked"
            marker.id = tid
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = tx
            marker.pose.position.y = ty
            marker.pose.position.z = tdz # 원래 높이
            
            marker.scale.x = tdx
            marker.scale.y = tdy
            marker.scale.z = tdz
            
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
            marker.color.a = 0.6
            marker.lifetime.nanosec = 200000000

            out_msg.markers.append(marker)

            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "id"
            text_marker.id = tid + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = tx
            text_marker.pose.position.y = ty
            text_marker.pose.position.z = tdz + 1.0
            text_marker.scale.z = 0.5
            text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0
            text_marker.text = f"ID:{tid}"
            out_msg.markers.append(text_marker)

        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()