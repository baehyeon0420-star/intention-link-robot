# intention-link-python

[intention-link](https://github.com/baehyeon0420-star/intention-link)의 실시간 EMG → 로봇팔 제어 파이프라인을 **Unity 없이 파이썬만으로** 재구현한 프로젝트.

## 왜 만들었나

기존 intention-link는 ESP32(MyoWare 2.0 EMG 센서) → 시리얼 → **Unity(C#)**가 신호를 분류하고 로봇팔 명령으로 변환하는 구조였음. 최종 목표(전완근 착용형 무선 EMG 유닛 + 로봇팔 완전 무선 연동)로 가는 과정에서 Unity/노트북 의존을 없애기로 하면서, 같은 로직을 파이썬으로 옮겨 담은 것.

기존 intention-link 저장소의 파일은 **하나도 수정/삭제하지 않았고**, 이 저장소는 완전히 별도의 새 프로젝트임.

## 원본(Unity)과의 대응 관계

| intention-link (C#) | intention-link-python | 역할 |
|---|---|---|
| `EMGSerialReader.cs` | `emg_pipeline/serial_reader.py` | 시리얼로 raw EMG 읽기, 정규화, threshold 상태(REST/LIGHT/STRONG/GRIP) 분류, ML용 200ms 윈도우 유지 |
| `EMGMLClassifier.cs` | `emg_pipeline/ml_classifier.py` | MAV/RMS/STD/WL/ZC 특징 추출 + LogisticRegression 추론 |
| `RobotCommandMapper.cs` | `emg_pipeline/command_mapper.py` (`map_to_robot_command`) | threshold 상태 → 로봇 명령(Release/Hold/GripClose) |
| `EMGStateManager.cs` | `emg_pipeline/command_mapper.py` (`ContractionPatternDetector`) | Short/Long/Double 수축 패턴 감지 (XR 제스처용, 현재 로봇 제어 경로에는 미사용) |
| `RobotArmController.cs` | `emg_pipeline/robot_controller.py` | 로봇 명령 실행 (Unity는 3D 비주얼 회전, 여기는 콘솔 출력 + 하드웨어 전송 훅) |
| `emg_data_collection/train.py`가 만든 `final_model.npz` | `final_model.npz` (원본에서 복사) | 학습된 스케일러/로지스틱회귀 계수 |

**중요**: 원본과 마찬가지로 실제 로봇 명령은 **threshold 기반 상태**로 내려지고, ML 분류기(`final_model.npz`)는 화면에 나란히 출력만 되는 비교/디버그용임 (`EMGMLClassifier.cs`가 Unity에서 하던 것과 동일).

## 사용법

```bash
pip install -r requirements.txt
python main.py --port /dev/cu.usbserial-XXXX
```

주요 옵션 (기본값은 `EMGSerialReader.cs`의 기본값과 동일):

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--port` | (필수) | ESP32 시리얼 포트 |
| `--baud` | 115200 | ESP32 `Serial.begin()`과 동일하게 |
| `--rest-baseline` | 18 | 힘 뺀 상태(rest) raw 평균값 — 사람마다 재보정 필요 |
| `--max-contraction` | 1321 | 최대 수축(grip) raw 평균값 — 사람마다 재보정 필요 |
| `--model` | `final_model.npz` | 학습된 모델 파일 경로 |
| `--interval` | 0.05 | 메인 루프 주기(초) |

ESP32 쪽 펌웨어는 기존 intention-link와 동일하게 `"t_ms,raw"` 형식(또는 raw 단독)으로 시리얼 출력하면 됨 (`emg_data_collection/collect.py`와 같은 포맷).

## 아직 안 된 것 / 다음 단계

- **로봇 하드웨어 연동**: `robot_controller.py`의 `_send_to_hardware()`는 아직 빈 훅. 실제 로봇팔(그리퍼) 제어보드의 통신 프로토콜이 정해지면 여기에 시리얼/PWM 전송 코드 추가
- **XR 상호작용**: `ContractionPatternDetector`(Short/Long/Double)는 이식만 해두고 실제로 연결한 곳은 없음. Unity의 XR 상호작용 기능은 이 프로젝트 범위 밖
- **무선화**: 지금은 여전히 유선 시리얼. 최종 목표(전완근 착용형 무선 유닛)를 위해서는 ESP32 → BLE/ESP-NOW 무선 전송으로 교체 필요 (그러면 이 파이썬 스크립트가 받는 지점도 시리얼 대신 BLE 수신으로 바뀌어야 함)
- **`rest-baseline`/`max-contraction` 자동 보정**: 지금은 수동으로 값을 넣어야 함

## 관련 프로젝트

- [intention-link](https://github.com/baehyeon0420-star/intention-link) — 원본 Unity 기반 프로젝트 (ML 모델 학습/데이터 수집 코드 포함)
- [energy-harvesting-research](https://github.com/baehyeon0420-star/energy-harvesting-research) — 저전력 설계 방법론 (웨어러블 EMG 유닛 배터리 설계에 적용 예정)
