import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment

class KalmanBoxTracker:
    """
    개별 객체 하나를 추적하는 칼만 필터 인스턴스
    시스템 모델: 등속도 모델 (Constant Velocity Model)
    """
    count = 0
    def __init__(self, bbox):
        # bbox: [x, y, z, dx, dy, dz, yaw, vx, vy, score, label] (nuScenes 11차원 기준)
        
        # 상태 벡터 (dim_x=7): [x, y, z, vx, vy, vz, yaw]
        # 관측 벡터 (dim_z=4): [x, y, z, yaw]
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        
        # 1. 상태 전이 행렬 (F) - 등속도 모델 적용
        # x_new = x + v * dt
        dt = 0.1  # 라이다 주기 (10Hz)
        self.kf.F = np.array([
            [1, 0, 0, dt, 0,  0,  0], # x  += vx * dt
            [0, 1, 0, 0,  dt, 0,  0], # y  += vy * dt
            [0, 0, 1, 0,  0,  dt, 0], # z  += vz * dt
            [0, 0, 0, 1,  0,  0,  0], # vx (유지)
            [0, 0, 0, 0,  1,  0,  0], # vy (유지)
            [0, 0, 0, 0,  0,  1,  0], # vz (유지)
            [0, 0, 0, 0,  0,  0,  1]  # yaw (유지)
        ])
        
        # 2. 관측 행렬 (H) - 위치와 각도만 관측됨
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0], # x
            [0, 1, 0, 0, 0, 0, 0], # y
            [0, 0, 1, 0, 0, 0, 0], # z
            [0, 0, 0, 0, 0, 0, 1]  # yaw
        ])

        # 3. 노이즈 공분산 행렬 설정 (튜닝 포인트)
        # R: 관측 노이즈 (센서가 얼마나 부정확한가?)
        self.kf.R[0:, 0:] *= 1.0   # 위치 노이즈
        self.kf.R[3, 3] *= 1.0     # 각도 노이즈

        # P: 초기 추정 오차 공분산 (처음 상태를 얼마나 확신하는가?)
        self.kf.P[4:, 4:] *= 1000.0 # 초기 속도는 모르니까 불확실성을 크게 둠
        self.kf.P *= 10.0

        # Q: 프로세스 노이즈 (시스템 모델이 얼마나 부정확한가?)
        # 등속도 모델이지만 실제로는 가감속하므로 속도 부분에 노이즈를 줌
        self.kf.Q[-1, -1] *= 0.01  # 각도 변화 노이즈
        self.kf.Q[4:, 4:] *= 0.01  # 속도 변화 노이즈

        # 4. 초기 상태 주입
        self.kf.x[:3] = bbox[:3].reshape(3, 1) # x, y, z
        self.kf.x[6] = bbox[6]                 # yaw
        
        # 트래커 관리 변수
        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        
        self.history = []
        self.hits = 0           # 총 매칭 횟수
        self.hit_streak = 0     # 연속 매칭 횟수
        self.age = 0            # 생성된 후 지난 시간
        
        # 크기(dx, dy, dz)는 필터링하지 않고 최신값 유지
        self.dimensions = bbox[3:6] 

        # [중요] Label(Class ID) 저장 (Index -1 사용)
        self.label = 0 
        if len(bbox) >= 1:
            self.label = int(bbox[-1]) # 맨 마지막 값을 라벨로 저장
        

    def update(self, bbox):
        """ [Correction] 모델이 관측한 값으로 상태 업데이트 """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        
        # 관측 벡터 z 만들기
        z = np.array([bbox[0], bbox[1], bbox[2], bbox[6]]).reshape(4, 1)
        self.kf.update(z)
        
        # 크기 정보 업데이트
        self.dimensions = bbox[3:6]

        # 라벨 업데이트 (혹시 바뀌었을 경우)
        if len(bbox) >= 1:
            self.label = int(bbox[-1])

    def predict(self):
        """ [Propagation] 등속도 모델을 이용해 다음 위치 예측 """
        # Yaw 각도 보정 (회전이 없다고 가정하거나 범위 제한)
        if((self.kf.x[6] + self.kf.x[2]) <= 0):
            self.kf.x[6] *= 0.0
            
        self.kf.predict()
        self.age += 1
        if(self.time_since_update > 0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(self.kf.x)
        return self.kf.x

    def get_state(self):
        """ [반환 포맷] 0~6: x, y, z, dx, dy, dz, yaw 7: ID 8,9: vx, vy (칼만필터 추정 속도) 10: Label """
        return [
            self.kf.x[0, 0], self.kf.x[1, 0], self.kf.x[2, 0],  # Pos
            self.dimensions[0], self.dimensions[1], self.dimensions[2], # Size
            self.kf.x[6, 0],  # Yaw
            self.id,          # ID
            self.kf.x[3, 0], self.kf.x[4, 0], # Velocity (vx, vy)
            self.label]

class SortTracker:
    """ 다중 객체 추적 매니저 (데이터 연관 담당) """
    def __init__(self, max_frames_to_skip=5, distance_threshold=2.0):
        self.trackers = []
        self.max_frames_to_skip = max_frames_to_skip # 이 횟수만큼 놓쳐도 살려둠
        self.distance_threshold = distance_threshold # 미터 단위 (Gating)

    def update(self, detections):
        # detections: (N, 7) [x, y, z, dx, dy, dz, yaw]
        
        # 1. 모든 트래커의 다음 위치 예측 (Propagation)
        # -> 모델이 놓쳐도 여기서 등속도 운동으로 위치가 갱신됨!
        for trk in self.trackers:
            trk.predict()

        # 2. 데이터 연관 (Data Association)
        # 예측된 트래커 위치 vs 이번 프레임 감지 위치 사이의 거리 행렬 계산
        if len(self.trackers) == 0:
            matched, unmatched_dets = [], np.arange(len(detections))
            unmatched_trks = []
        else:
            dist_matrix = np.zeros((len(detections), len(self.trackers)))
            for d, det in enumerate(detections):
                for t, trk in enumerate(self.trackers):
                    # 트래커의 예측 위치(trk.kf.x)와 디텍션(det) 사이 거리
                    dist_matrix[d, t] = np.linalg.norm(det[:3] - trk.kf.x[:3, 0])

            # 헝가리안 알고리즘으로 최적 매칭 수행
            row_ind, col_ind = linear_sum_assignment(dist_matrix)
            
            matched = []
            unmatched_dets = []
            unmatched_trks = []
            
            # Gating: 거리가 너무 멀면 매칭 취소
            for d, t in zip(row_ind, col_ind):
                if dist_matrix[d, t] > self.distance_threshold:
                    unmatched_dets.append(d)
                    unmatched_trks.append(t)
                else:
                    matched.append((d, t))
            
            # 헝가리안 알고리즘 결과에 포함되지 않은 인덱스 분류
            for d in range(len(detections)):
                if d not in row_ind: unmatched_dets.append(d)
            for t in range(len(self.trackers)):
                if t not in col_ind: unmatched_trks.append(t)

        # 3. 매칭된 트래커 업데이트 (Correction)
        for d, t in matched:
            self.trackers[t].update(detections[d])

        # 4. 매칭 안 된 새 박스 -> 신규 트래커 생성
        for d in unmatched_dets:
            trk = KalmanBoxTracker(detections[d])
            self.trackers.append(trk)

        # 5. 죽은 트래커 삭제 및 결과 출력
        ret = []
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state() 
            
            # 최소 1번은 매칭됐거나(hit), 방금 태어난 녀석이면서 아직 살아있다면 출력
            # time_since_update <= max_frames_to_skip 조건 덕분에 
            # 모델이 몇 프레임 놓쳐도 트래커는 계속 살아남아 위치를 예측해서 뱉어줌!
            if (trk.time_since_update < 1) or (trk.hit_streak >= 1 and trk.time_since_update <= self.max_frames_to_skip):
                ret.append(d) 
            
            i -= 1
            # 수명 다하면 삭제
            if (trk.time_since_update > self.max_frames_to_skip):
                self.trackers.pop(i)
                
        if len(ret) > 0: return np.array(ret)
        return np.empty((0, 11))