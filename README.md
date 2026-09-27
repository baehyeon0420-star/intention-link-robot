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

**중요**: 실제 로봇 명령은 **2상태**(`GripLatch`: STRONG/GRIP→GripClose, REST→Release, LIGHT→이전 명령 유지)로 내려짐 (2026-09-22). 4단계 threshold 상태와 ML 분류기(`final_model.npz`)는 화면에 나란히 출력만 되는 비교/디버그용임.

**보정 기준 (2026-09-22 변경)**: norm=1.0 은 "최대한 세게 쥔 값"이 아니라 **"물건 집듯이 편하게 쥔 값"**(`--ref-contraction`)임. 보정 안내도 "편하게 쥐세요"로 바뀜. 세게 쥘 필요 없이 편한 세기의 70%(`--threshold-strong`, 2026-09-27 0.65→0.7)에서 쥐고, 거의 다 힘을 뺄 때까지(REST) 유지한다.

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

# 매번 전극 접촉 상태가 달라지므로 --calibrate로 자동 보정 추천
# (시작하면 "힘 빼기 3초" -> "물건 집듯이 편하게 쥐기 3초" 순서로 안내가 뜸, 결과는 calibration.json 저장)
python main.py --port /dev/cu.usbserial-XXXX --calibrate

# 같은 전극 상태에서 다시 실행할 때만: 저장된 보정값 재사용
python main.py --port /dev/cu.usbserial-XXXX --use-saved

# EMG + AmazingHand 실제 구동
python main.py --port /dev/cu.usbserial-XXXX --calibrate --hand-port /dev/cu.usbmodemXXXX
```

주요 옵션 (기본값은 `EMGSerialReader.cs`의 기본값과 동일):

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--port` | (필수) | ESP32(EMG) 시리얼 포트 |
| `--baud` | 115200 | ESP32 `Serial.begin()`과 동일하게 |
| `--rest-baseline` | 18 | 힘 뺀 상태 raw 중앙값. 임시 기본값 — `--calibrate` 로 재보정 |
| `--ref-contraction` | 400 | 편하게 쥔 raw 중앙값 (norm=1.0). 임시 기본값 — `--calibrate` 로 재보정 |
| `--max-contraction` | (없음) | 구버전 옵션. 주면 `--ref-contraction` 으로 취급 |
| `--threshold-strong` | 0.65 | 쥐기 진입 기준 (편한 쥐기 대비 비율). `analysis/compare_thresholds.py` 로 재조정 |
| `--threshold-light` | 0.2 | LIGHT 진입 기준. LIGHT 에서는 이전 명령 유지 |
| `--release-ratio` | 0.43 | 상태 하강 기준 = threshold × 이 값 (히스테리시스) |
| `--calibrate` | 꺼짐 | 시작할 때 힘 빼기 3초 + 편하게 쥐기 3초를 측정해 rest/ref 를 중앙값으로 잡고 `calibration.json` 에 저장. 상승폭이 rest 잡음의 3배 미만이면 전극 확인 경고 후 재시도 |
| `--use-saved` | 꺼짐 | `calibration.json` 의 보정값 재사용. 전극을 다시 붙였다면 쓰지 말 것 |
| `--calibrate-seconds` | 3.0 | 캘리브레이션 각 단계 측정 시간(초) |
| `--model` | `final_model.npz` | 학습된 모델 파일 경로 |
| `--interval` | 0.05 | 메인 루프 주기(초) |
| `--hand-port` | (없음) | AmazingHand용 USB-TTL 포트. 안 주면 콘솔 출력만 하고 손은 안 움직임 |
| `--smoothing` | 0.02 | EMG 평활 강도. 500Hz에서 시간상수 약 100ms |

## 오프라인 분석

```bash
# 보정 방식 비교 (피크x0.9 vs 중앙값+히스테리시스, 4단계 경로)
cd emg_data_collection && python3 calibration_compare.py --min-rep-ratio 0.4 --rest-window-max 500

# 판정 기준 비교 (A 고정 / B 편한 쥐기 보정 / C 최대 수축 보정, k 0.3~0.9 스캔, 2상태 래치 경로)
python3 analysis/compare_thresholds.py --min-rep-ratio 0.4 --k 0.7   # → analysis/out/ 에 md 표·csv·png
```
피험자 데이터는 `collect.py` 로 rest / grip / light 를 받는다 (light = "물건 집듯이 편하게" 3초x6회).

## 아직 안 된 것 / 다음 단계

- **HOLD 동작 다듬기**: 지금은 Release/GripClose 각도의 단순 중간값(`amazinghand_driver.py`의 `HOLD_DEG`). 실제로 써보고 조정 필요
- **판정 기준(k) 확정**: 편한 쥐기 기준 0.7/0.4 는 초기값. `analysis/compare_thresholds.py` 로 피험자 데이터에서 k 를 정할 것
- **XR 상호작용**: `ContractionPatternDetector`(Short/Long/Double)는 이식만 해두고 실제로 연결한 곳은 없음. Unity의 XR 상호작용 기능은 이 프로젝트 범위 밖
- **무선화**: 지금은 여전히 유선 시리얼. 최종 목표(전완근 착용형 무선 유닛)를 위해서는 ESP32 → BLE/ESP-NOW 무선 전송으로 교체 필요 (그러면 이 파이썬 스크립트가 받는 지점도 시리얼 대신 BLE 수신으로 바뀌어야 함)

## 관련 프로젝트

- [intention-link](https://github.com/baehyeon0420-star/intention-link) — 원본 Unity 기반 프로젝트 (ML 모델 학습/데이터 수집 코드 포함)
- [energy-harvesting-research](https://github.com/baehyeon0420-star/energy-harvesting-research) — 저전력 설계 방법론 (웨어러블 EMG 유닛 배터리 설계에 적용 예정)
