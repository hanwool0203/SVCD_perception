import numpy as np
import open3d as o3d
import os

# 방금 생성한 5-sweep 폴더 경로
DATA_PATH = '../OpenPCDet/data/custom/points_5sweep/'

# 확인할 파일 (중간쯤 있는 거 아무거나)
FILE_NAME = '000300.npy' 

full_path = os.path.join(DATA_PATH, FILE_NAME)

if not os.path.exists(full_path):
    print(f"파일이 없습니다: {full_path}")
    exit()

# 1. NPY 파일 읽기
points = np.load(full_path)
print(f"파일 로드: {FILE_NAME}")
print(f"점 개수: {len(points)}")
print(f"데이터 형태: {points.shape}") # (N, 5) 예상

# 2. Open3D 포인트 클라우드로 변환
pcd = o3d.geometry.PointCloud()
# [x, y, z] 좌표만 넣기
pcd.points = o3d.utility.Vector3dVector(points[:, :3])

# 3. 색상 입히기 (높이 또는 Intensity 기준)
# 여기서는 높이(z)에 따라 색을 입혀서 보기 쉽게 함
z = points[:, 2]
max_z, min_z = np.max(z), np.min(z)
colors = np.zeros((len(z), 3))
colors[:, 0] = (z - min_z) / (max_z - min_z) # Red 채널에 높이 반영
colors[:, 1] = 1.0 - (z - min_z) / (max_z - min_z) # Green 채널 반대
pcd.colors = o3d.utility.Vector3dVector(colors)

# 4. 시각화 창 띄우기
print("시각화 창이 열립니다. 마우스로 돌려보세요.")
print("확인 포인트: 벽이나 기둥이 '칼같이' 하나로 보이는지, 아니면 '잔상'처럼 여러 개로 겹쳐 보이는지 확인하세요.")
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='5-Sweep Check', width=1024, height=768)
vis.add_geometry(pcd)

# 렌더링 옵션 가져오기
opt = vis.get_render_option()
opt.point_size = 1.0  # 점 크기를 2.0으로 설정 (기본값은 보통 5.0이라 큼)
opt.background_color = np.asarray([0, 0, 0]) # 배경을 검은색으로 (취향껏 [1, 1, 1]로 흰색 가능)

print("시각화 창이 열렸습니다.")
print("- 점이 여전히 크다면 키보드 '-' 키를 누르세요.")
print("- 배경을 바꾸고 싶다면 코드에서 background_color를 수정하세요.")

vis.run()
vis.destroy_window()