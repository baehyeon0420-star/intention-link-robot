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
| `RobotArmController.cs` | `emg_pipeline/robot_controller.py` + `emg_pipeline/amazinghand_driver.py` | 로봇 명령 실행. 실제 하드웨어는 [AmazingHand](https://github.com/pollen-robotics/AmazingHand)(Feetech SCS0009 x8, rustypot)로 연결됨 |
| `emg_data_collection/train.py`가 만든 `final_model.npz` | `final_model.npz` (원본에서 복사) | 학습된 스케일러/로지스틱회귀 계수 |

**중요**: 원본과 마찬가지로 실제 로봇 명령은 **threshold 기반 상태**로 내려지고, ML 분류기(`final_model.npz`)는 화면에 나란히 출력만 되는 비교/디버그용임 (`EMGMLClassifier.cs`가 Unity에서 하던 것과 동일).

## 하드웨어 연결

### 1. EMG 센서(MyoWare 2.0) ↔ ESP32

| MyoWare 2.0 핀 | ESP32 핀 |
|---|---|
| `VIN` | 3.3V (5V 아님 — ENV 출력이 0~VIN이라 5V로 물리면 ESP32 ADC가 손상될 수 있음) |
| `GND` | GND |
| `ENV` | GPIO34 (ADC1, 입력 전용 핀) |

전극 부착: `+`/`-` 전극 2개는 전완근(손 쥘 때 쓰는 아래팔 안쪽 근육) 위에 근육 결 방향으로 나란히, `REF` 전극 1개는 손목뼈/팔꿈치처럼 근육 없는 부위에.

ESP32에는 `esp32_firmware/emg_myoware_reader/emg_myoware_reader.ino`를 올릴 것. `"t_ms,raw\n"` 형식으로 115200bps 시리얼 출력함 (`emg_data_collection/collect.py`와 동일 포맷).

### 2. 로봇 손(AmazingHand) ↔ 컴퓨터

USB-TTL 시리얼 버스 드라이버로 연결 (ESP32와는 **별도의 USB 포트**). 서보 ID 설정/캘리브레이션은 [amazinghand-setup](https://github.com/baehyeon0420-star/amazinghand-setup) 참고.

## 사용법

```bash
pip install -r requirements.txt

# EMG만 (로봇 손 없이 콘솔 출력만 확인)
python main.py --port /dev/cu.usbserial-XXXX

# EMG + AmazingHand 실제 구동
python main.py --port /dev/cu.usbserial-XXXX --hand-port /dev/cu.usbmodemXXXX
```

주요 옵션 (기본값은 `EMGSerialReader.cs`의 기본값과 동일):

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--port` | (필수) | ESP32(EMG) 시리얼 포트 |
| `--baud` | 115200 | ESP32 `Serial.begin()`과 동일하게 |
| `--rest-baseline` | 18 | 힘 뺀 상태(rest) raw 평균값 — 사람마다 재보정 필요 |
| `--max-contraction` | 1321 | 최대 수축(grip) raw 평균값 — 사람마다 재보정 필요 |
| `--model` | `final_model.npz` | 학습된 모델 파일 경로 |
| `--interval` | 0.05 | 메인 루프 주기(초) |
| `--hand-port` | (없음) | AmazingHand용 USB-TTL 포트. 안 주면 콘솔 출력만 하고 손은 안 움직임 |

## 아직 안 된 것 / 다음 단계

- **HOLD 동작 다듬기**: 지금은 Release/GripClose 각도의 단순 중간값(`amazinghand_driver.py`의 `HOLD_DEG`). 실제로 써보고 조정 필요
- **XR 상호작용**: `ContractionPatternDetector`(Short/Long/Double)는 이식만 해두고 실제로 연결한 곳은 없음. Unity의 XR 상호작용 기능은 이 프로젝트 범위 밖
- **무선화**: 지금은 여전히 유선 시리얼. 최종 목표(전완근 착용형 무선 유닛)를 위해서는 ESP32 → BLE/ESP-NOW 무선 전송으로 교체 필요 (그러면 이 파이썬 스크립트가 받는 지점도 시리얼 대신 BLE 수신으로 바뀌어야 함)
- **`rest-baseline`/`max-contraction` 자동 보정**: 지금은 수동으로 값을 넣어야 함

## 관련 프로젝트

- [intention-link](https://github.com/baehyeon0420-star/intention-link) — 원본 Unity 기반 프로젝트 (ML 모델 학습/데이터 수집 코드 포함)
- [energy-harvesting-research](https://github.com/baehyeon0420-star/energy-harvesting-research) — 저전력 설계 방법론 (웨어러블 EMG 유닛 배터리 설계에 적용 예정)
