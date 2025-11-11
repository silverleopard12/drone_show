# Neural Network Models

이 디렉토리는 충돌 예측을 위한 신경망 모델 파일을 저장합니다.

## 모델 형식

지원 예정 형식:
- PyTorch (`.pt`, `.pth`)
- ONNX (`.onnx`)
- TensorFlow SavedModel
- 커스텀 형식 (`.weights`)

## 모델 구조

### Collision Prediction Network

**입력**:
- 드론 1 상태: 위치 (x, y, z), 속도 (vx, vy, vz)
- 드론 2 상태: 위치 (x, y, z), 속도 (vx, vy, vz)
- 총 12차원

**출력**:
- 충돌 확률: [0, 1]
- 1차원

**아키텍처 예시**:
```
Input(12) -> Dense(64, ReLU) -> Dense(32, ReLU) -> Dense(16, ReLU) -> Dense(1, Sigmoid)
```

## 모델 학습

### 데이터 생성

```python
# TODO: 학습 데이터 생성 스크립트
# scripts/generate_training_data.py
```

### 학습

```python
# TODO: 모델 학습 스크립트
# scripts/train_collision_net.py
```

## 모델 변환

PyTorch → ONNX 변환 예시:
```python
import torch
import torch.onnx

model = CollisionNet()
model.load_state_dict(torch.load('collision_net.pt'))
model.eval()

dummy_input = torch.randn(1, 12)
torch.onnx.export(model, dummy_input, 'collision_net.onnx',
                  input_names=['state'],
                  output_names=['collision_prob'])
```

## 사전 학습 모델

현재 사전 학습된 모델은 제공되지 않습니다.
자신의 시나리오에 맞는 데이터로 모델을 학습해야 합니다.

## 모델 배치

학습된 모델을 이 디렉토리에 배치하고,
`config/default.yaml`의 `model_path` 파라미터를 업데이트하세요.

```yaml
dl_planner:
  ros__parameters:
    model_path: "models/your_model.pt"
```
