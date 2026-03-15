import numpy as np
import os
import open3d as o3d
from collections import deque
import copy

# ================= [사용자 설정] =================
# 아까 simple_converter.py로 만든 1-sweep 데이터 폴더
INPUT_DIR = '../OpenPCDet/data/custom/points/'
# 5-sweep 결과물이 저장될 폴더
OUTPUT_DIR = '../OpenPCDet/data/custom/points_5sweep/'

MAX_SWEEPS = 5
VOXEL_SIZE = 0.3  # ICP 속도 향상을 위한 다운샘플링 크기 (m)
# ===============================================

os.makedirs(OUTPUT_DIR, exist_ok=True)

# 1. 파일 목록 가져오기
# 000000.npy 순서대로 정렬
npy_files = sorted([f for f in os.listdir(INPUT_DIR) if f.endswith('.npy')])
print(f"📂 총 {len(npy_files)}개의 파일을 처리합니다.")

# 버퍼: {'points': (N,4), 'pose': (4,4), 'idx': int}
sweep_buffer = deque(maxlen=MAX_SWEEPS)

# 전역 포즈 (Global Pose): 월드 좌표계 기준 현재 차량 위치
current_global_pose = np.eye(4)

def get_pcd_from_numpy(np_points):
    """Numpy -> Open3D PointCloud 변환"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_points[:, :3])
    return pcd

def preprocess_point_cloud(pcd, voxel_size):
    """ICP 정합을 위해 점을 솎아내고(Downsample) 법선(Normal) 계산"""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30))
    return pcd_down

# 이전 프레임 데이터 저장용
prev_pcd_down = None

print("🚀 5-Sweep 변환 시작 (Open3D ICP Mode)...")

for idx, filename in enumerate(npy_files):
    
    # 2. 파일 읽기
    file_path = os.path.join(INPUT_DIR, filename)
    points_np = np.load(file_path) # (N, 4) [x,y,z,i]
    
    # 3. Open3D 변환 및 전처리
    curr_pcd = get_pcd_from_numpy(points_np)
    curr_pcd_down = preprocess_point_cloud(curr_pcd, VOXEL_SIZE)
    
    # 4. ICP (Pose 계산)
    if prev_pcd_down is not None:
        # [Point-to-Plane ICP]
        # Source(현재) -> Target(과거)로 맞추는 변환 행렬 계산
        # 초기값(init)은 Identity(움직임 없음)로 가정
        reg_p2l = o3d.pipelines.registration.registration_icp(
            curr_pcd_down, prev_pcd_down, 1.0, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )
        
        # T_curr_to_prev
        transformation = reg_p2l.transformation
        
        # Global Pose 누적: T_world_curr = T_world_prev * T_curr_to_prev
        # (행렬 곱 순서 주의: Open3D는 Pre-multiply 방식이 일반적)
        current_global_pose = np.matmul(current_global_pose, transformation)
        
    # 다음 루프를 위해 현재를 과거로 저장
    prev_pcd_down = curr_pcd_down
    
    # 5. 버퍼에 저장 (원본 포인트 + 계산된 Pose)
    sweep_buffer.append({
        'points': points_np,
        'pose': current_global_pose.copy(),
        'frame_idx': idx
    })
    
    # 6. 스택킹 (Stacking) & 저장
    # 버퍼에 있는 모든 프레임을 현재 프레임 좌표계로 변환해서 합치기
    if len(sweep_buffer) > 0:
        merged_points = []
        
        # T_world_curr의 역행렬 (Global -> Current Local)
        curr_pose_inv = np.linalg.inv(current_global_pose)
        
        for sweep in sweep_buffer:
            past_points = sweep['points'] # (N, 4)
            past_pose = sweep['pose']     # Global Pose
            
            # 좌표 변환: T_relative = T_curr_inv * T_past
            # 과거 데이터를 현재 차의 위치 기준으로 가져옴
            rel_transform = np.matmul(curr_pose_inv, past_pose)
            
            # 회전 및 이동 적용: P_new = R * P + T
            xyz = past_points[:, :3]
            xyz_transformed = (rel_transform[:3, :3] @ xyz.T).T + rel_transform[:3, 3]
            
            # Time Lag 계산 (10Hz 가정 -> 프레임당 0.1초 차이)
            # 현재 idx - 과거 idx
            time_lag = (idx - sweep['frame_idx']) * 0.1
            if time_lag < 0: time_lag = 0.0
            
            # (N, 5) 생성 [x, y, z, intensity, time_lag]
            # intensity는 그대로 사용
            num_pts = len(xyz_transformed)
            points_5d = np.zeros((num_pts, 5), dtype=np.float32)
            
            points_5d[:, :3] = xyz_transformed.astype(np.float32)
            points_5d[:, 3] = past_points[:, 3]
            points_5d[:, 4] = time_lag
            
            merged_points.append(points_5d)
        
        # 합치기
        final_data = np.vstack(merged_points)
        
        # 저장
        save_path = os.path.join(OUTPUT_DIR, filename)
        np.save(save_path, final_data)
        
        if idx % 50 == 0:
            print(f"✅ Saved {filename} (Sweeps: {len(sweep_buffer)}, Points: {len(final_data)})")

print("🎉 5-Sweep 변환 완료!")
print(f"💾 저장 경로: {OUTPUT_DIR}")