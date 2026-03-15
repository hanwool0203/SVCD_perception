import numpy as np
import os
from pathlib import Path

# ROS2 bag 읽기용 라이브러리
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# ================= [사용자 설정] =================
BAG_PATH = '../bag/2.20_/2.20_4'     # .mcap 또는 .db3 파일이 있는 폴더 경로
OUTPUT_DIR = '../OpenPCDet_old/data/custom/points/'
LIDAR_TOPIC = '/velodyne_points'
MIN_DISTANCE = 1.0                # 차량 본체 등 너무 가까운 점 제거 (미터)
# ===============================================

# 저장 폴더 생성
os.makedirs(OUTPUT_DIR, exist_ok=True)
file_idx = 0

# ROS2 표준 타입 스토어 생성
typestore = get_typestore(Stores.LATEST)

# --- PointCloud2 파싱 함수 (검증된 안전 버전) ---
def parse_pointcloud2(msg):
    """
    ROS2 PointCloud2 메시지 -> Numpy Array (N, 4) [x, y, z, intensity]
    """
    fields = {f.name: f for f in msg.fields}
    
    # 필수 필드 체크
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        return np.array([])
        
    offset_x = fields['x'].offset
    offset_y = fields['y'].offset
    offset_z = fields['z'].offset
    
    offset_i = -1
    if 'intensity' in fields: offset_i = fields['intensity'].offset
    elif 'i' in fields: offset_i = fields['i'].offset
    
    point_step = msg.point_step
    width = msg.width
    height = msg.height
    num_points = width * height
    
    # 바이너리 데이터 추출
    if not isinstance(msg.data, (bytes, bytearray, memoryview)):
        data_buffer = np.array(msg.data, dtype=np.uint8).tobytes()
    else:
        data_buffer = msg.data
    
    # 결과 배열 생성 (N, 4)
    points = np.zeros((num_points, 4), dtype=np.float32)
    
    # 데이터 파싱 (Strided View)
    points[:, 0] = np.ndarray(shape=(num_points,), dtype=np.float32, buffer=data_buffer, offset=offset_x, strides=(point_step,))
    points[:, 1] = np.ndarray(shape=(num_points,), dtype=np.float32, buffer=data_buffer, offset=offset_y, strides=(point_step,))
    points[:, 2] = np.ndarray(shape=(num_points,), dtype=np.float32, buffer=data_buffer, offset=offset_z, strides=(point_step,))
    
    if offset_i >= 0:
        points[:, 3] = np.ndarray(shape=(num_points,), dtype=np.float32, buffer=data_buffer, offset=offset_i, strides=(point_step,))
    
    return points

# --- 메인 프로세싱 ---
print(f"🚀 단순 변환기 시작 (Single Sweep Mode)")
print(f"📂 대상: {BAG_PATH}")

with AnyReader([Path(BAG_PATH)], default_typestore=typestore) as reader:
    
    # 1. 토픽 연결 확인
    connections = [x for x in reader.connections if x.topic == LIDAR_TOPIC]
    if not connections:
        print(f"❌ Error: 토픽 '{LIDAR_TOPIC}'을 찾을 수 없습니다!")
        exit()
        
    total_msgs = reader.message_count
    print(f"📊 총 메시지 수: {total_msgs}")
    
    # 2. 메시지 순회
    for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=connections)):
        
        if i % 100 == 0:
            print(f"Processing frame {i}/{total_msgs}...")

        # 3. 메시지 디시리얼라이즈
        msg = reader.deserialize(rawdata, connection.msgtype)
        
        # 4. 파싱 (바이너리 -> Numpy)
        points_np = parse_pointcloud2(msg)
        if len(points_np) == 0: continue

        # 5. [전처리] 거리 필터링 & NaN 제거
        # (단순하고 강력한 필터링)
        
        # NaN 제거
        mask = np.isfinite(points_np[:, :3]).all(axis=1)
        clean_points = points_np[mask]
        
        # 거리 계산 및 필터링
        xyz = clean_points[:, :3]
        dist = np.linalg.norm(xyz, axis=1)
        
        # 최소 거리(차체) ~ 최대 거리(센서 한계) 사이만 남김
        valid_mask = (dist > MIN_DISTANCE) & (dist < 100.0)
        final_points = clean_points[valid_mask]
        
        if len(final_points) == 0: continue
        
        # 6. 저장 (N, 4) -> [x, y, z, intensity]
        # 타임스탬프 채널 없음 (Single Sweep이므로 불필요)
        save_path = os.path.join(OUTPUT_DIR, f"{file_idx:06d}.npy")
        np.save(save_path, final_points)
        
        file_idx += 1

print(f"🎉 변환 완료! 총 {file_idx}개의 파일이 생성되었습니다.")
print(f"💾 저장 경로: {OUTPUT_DIR}")