import zmq
import numpy as np
import torch
import time
import sys
from pathlib import Path
import os
import traceback

PROJECT_ROOT = Path('/home/omen16/workspace/OpenPCDet_old') 
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

TOOLS_DIR = PROJECT_ROOT / 'tools'
if os.path.exists(TOOLS_DIR):
    os.chdir(TOOLS_DIR)

# OpenPCDet 라이브러리 로드
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from pcdet.datasets import DatasetTemplate

# =========================================================
# [사용자 설정] 경로를 본인 환경에 맞게 수정하세요!
# =========================================================
# CFG_FILE = str(PROJECT_ROOT / 'tools/cfgs/nuscenes_models/cbgs_voxel0075_voxelnext.yaml')
# CKPT_FILE = str(PROJECT_ROOT / 'tools/ckpt/nuscenes/voxelnext_nuscenes_kernel1.pth')
# CFG_FILE = str(PROJECT_ROOT / 'tools/cfgs/kitti_models/pv_rcnn_cqca.yaml')
# CKPT_FILE = str(PROJECT_ROOT / 'tools/ckpt/kitti/pv_rcnn_cqca.pth')
# CFG_FILE = str(PROJECT_ROOT / 'tools/cfgs/kitti_models/pointpillar_SPA_CQCA.yaml')
# CKPT_FILE = str(PROJECT_ROOT / 'tools/ckpt/kitti/pointpillar_CQCA.pth')
CFG_FILE = str(PROJECT_ROOT / 'tools/cfgs/nuscenes_models/cbgs_voxel0075_voxelnext_cqca.yaml')
CKPT_FILE = str(PROJECT_ROOT / 'tools/ckpt/nuscenes/voxelnext_16ch_base.pth')
SCORE_THRESH = 0.66
# =========================================================
class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=False, root_path=None, logger=None, ext='.bin'):
        """
        Args:
            root_path: root path of dataset
            dataset_cfg: dataset config
            class_names: class names of dataset
            training: training mode
            logger: logger
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
def build_pcdet_model():

    print("🧠 [Server] 모델 설정을 로드합니다...")
    cfg_from_yaml_file(CFG_FILE, cfg)
    
    logger = common_utils.create_logger()

    print("🧠 [Server] 데이터셋 템플릿을 생성합니다...")
    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        training=False,
        root_path=Path('/tmp'),
        ext='.bin',
        logger=logger
    )
    
    print("🧠 [Server] 네트워크를 구축합니다...")
    model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    
    print(f"🧠 [Server] 가중치 파일을 로드합니다: {CKPT_FILE}")
    model.load_params_from_file(filename=CKPT_FILE, logger=logger)
    
    model.cuda()
    model.eval()
    print("🧠 [Server] 모델 준비 완료!")
    
    # [수정] 모델과 데이터셋 객체를 둘 다 리턴합니다.
    return model, demo_dataset

def main():
    # 1. 모델 로드 (시간이 좀 걸림)
    model, demo_dataset= build_pcdet_model()

    # [중요] 모델이 기대하는 채널 수 확인 (보통 NuScenes는 5)
    expected_dim = len(cfg.DATA_CONFIG.POINT_FEATURE_ENCODING.src_feature_list)
    print(f"🧠 [Info] 모델이 기대하는 입력 차원 수: {expected_dim}")
    
    # 2. ZMQ 서버 설정
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")

    print("🚀 [Server] 추론 준비 완료! 데이터 대기 중...")

    with torch.no_grad():
            while True:
                try:
                    # --- (A) 데이터 수신 ---
                    # 여기서 에러가 나면 ZMQ 상태가 꼬이지 않지만,
                    # 데이터를 받은 직후 에러가 나면 반드시 응답을 줘야 합니다.
                    meta = socket.recv_json()
                    msg = socket.recv()

                    # NumPy 배열로 복원
                    points = np.frombuffer(msg, dtype=meta['dtype'])
                    points = points.reshape(meta['shape'])

                    # [안전장치] 점이 너무 적으면(예: 0개) 추론 건너뛰기
                    if points.shape[0] == 0:
                        raise ValueError("수신된 포인트가 0개입니다.")
                    
                    # =====================================================
                    # [핵심 수정] 차원 맞추기 (Padding)
                    # (N, 4) -> (N, 5) 로 변환 (마지막에 timestamp=0 추가)
                    # =====================================================
                    current_dim = points.shape[1]
                    if current_dim < expected_dim:
                        # 부족한 만큼 0으로 채운 컬럼 생성
                        diff = expected_dim - current_dim
                        padding = np.zeros((points.shape[0], diff), dtype=points.dtype)
                        
                        # 옆에 갖다 붙이기 (Horizontal Stack)
                        # 결과: [x, y, z, i] + [0] -> [x, y, z, i, 0]
                        points = np.hstack([points, padding])
                    
                    # ====================================================

                    # --- (B) [수정] 전처리 (Voxelization) ---
                    # 이제 수동으로 텐서를 만들지 않고, dataset의 기능을 이용합니다.
                    
                    # 1. Raw Numpy 데이터를 dict로 포장
                    input_dict = {
                        'points': points, 
                        'frame_id': 'ros_frame'
                    }

                    # 2. OpenPCDet 내부 처리 파이프라인 실행 (여기서 voxels가 생성됨!)
                    data_dict = demo_dataset.prepare_data(data_dict=input_dict)
                    
                    # 3. 배치(Batch) 형태로 묶기 (List -> Batch Tensor)
                    data_dict = demo_dataset.collate_batch([data_dict])
                    
                    # 4. 데이터를 GPU로 이동
                    load_data_to_gpu(data_dict)

                    # --- (C) 추론 수행 ---
                    # model()은 (pred_dicts, recall_dict) 튜플을 반환합니다.
                    # [0]을 하면 pred_dicts(리스트)를 가져옵니다.
                    preds_list = model(data_dict)[0] 
                    
                    # 배치 사이즈가 1이므로, 리스트의 첫 번째 요소([0])가 우리가 원하는 결과 딕셔너리입니다.
                    result_dict = preds_list[0]

                    # --- (D) 후처리 (박스 필터링) ---
                    # 이제 result_dict는 딕셔너리이므로 키값으로 접근 가능합니다.
                    pred_boxes = result_dict['pred_boxes'].cpu().numpy()
                    pred_scores = result_dict['pred_scores'].cpu().numpy()
                    pred_labels = result_dict['pred_labels'].cpu().numpy()

                    # Score Threshold 적용
                    mask = pred_scores > SCORE_THRESH
                    final_boxes = pred_boxes[mask]
                    final_scores = pred_scores[mask]
                    final_labels = pred_labels[mask]

                    # [수정] 11차원 데이터 만들기 (Box + Score + Label)
                    # hstack을 위해 shape을 (N, 1)로 맞춰줌
                    final_scores_v = final_scores.reshape(-1, 1)
                    final_labels_v = final_labels.reshape(-1, 1)

                    # (N, 9) + (N, 1) + (N, 1) -> (N, 11)
                    final_data = np.hstack([final_boxes, final_scores_v, final_labels_v])

                    class_names = cfg.CLASS_NAMES # config에서 클래스 이름 목록 가져오기
                    if len(final_boxes) > 0:
                        print(f"✅ 감지됨: {len(final_boxes)}개")
                        for i, box in enumerate(final_boxes):
                            # 클래스 ID는 1부터 시작하므로 -1 해줌
                            cls_name = class_names[final_labels[i] - 1] 
                            print(f"   - 물체 {i}: {cls_name} (크기: {box[3]:.2f}, {box[4]:.2f}, {box[5]:.2f})")
                        
                    # --- (E) 결과 전송 ---
                    reply_meta = {
                        'dtype': str(final_data.dtype),
                        'shape': final_data.shape
                    }
                    socket.send_json(reply_meta, flags=zmq.SNDMORE)
                    socket.send(final_data.tobytes())

                except Exception as e:
                    # 1. 진짜 에러 내용을 출력 (Traceback)
                    print(f"\n❌ [Processing Error] 처리 중 에러 발생:")
                    traceback.print_exc() # 이게 있어야 진짜 원인을 알 수 있음!

                    # 2. [중요] 에러가 나도 클라이언트에게 '빈 응답'을 보내서 상태 리셋
                    try:
                        empty_boxes = np.zeros((0, 7), dtype=np.float32)
                        reply_meta = {
                            'dtype': str(empty_boxes.dtype),
                            'shape': empty_boxes.shape
                        }
                        socket.send_json(reply_meta, flags=zmq.SNDMORE)
                        socket.send(empty_boxes.tobytes())
                        print("⚠️ 빈 응답 전송 완료 (ZMQ 상태 복구)")
                    except Exception as z_e:
                        print(f"💀 ZMQ 복구 실패: {z_e}")
                    
                    continue

if __name__ == "__main__":
    main()
