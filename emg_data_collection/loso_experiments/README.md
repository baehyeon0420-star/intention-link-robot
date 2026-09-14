# LOSO 검증 실험 (결과보고서 데이터, 100Hz)

결과보고서 시점의 원본 데이터(s1~s4, 4명)와 그 위에서 돌린 검증 스크립트.
**이 폴더의 CSV는 100Hz 펌웨어로 받은 것**이라 상위 폴더의 500Hz 데이터(`s1_*_v5~v7.csv`)와 직접 비교하면 안 된다.

## 파일

| 파일 | 역할 |
|---|---|
| `s{1..4}_{rest,grip}.csv` | 4명 × 휴식/집기 원본 (`t_ms,raw,subject` 컬럼) |
| `features.py` | 200ms(20샘플) 창, 50% 겹침 → MAV/RMS/STD/WL/ZC 5특징 → `X.npy`, `y.npy`, `groups.npy` |
| `train.py` | 랜덤 분할 vs LOSO 비교 → 전체 4명으로 최종 LogisticRegression 학습 → `final_model.npz` (저장소 루트의 것과 동일) |
| `analyze.py` | CSV별 기초 통계 |
| `mvc_norm_experiment.py` | %MVC 정규화 / few-shot 개인보정 / 주파수 특징 추가 실험 |
| `advanced_validation.py` | 학습만 라벨 정제 / 정제 비율 민감도 / few-shot 곡선 |

## 실행

```bash
cd emg_data_collection/loso_experiments
python3 features.py
python3 train.py
python3 mvc_norm_experiment.py
python3 advanced_validation.py
```

## 결과 (2026-09-14 재현)

| 항목 | LogReg | RF |
|---|---|---|
| 랜덤 분할 (같은 사람 섞임) | 91.5% | – |
| **LOSO 기준선 (한 명 완전히 빼고 평가)** | **86.9%** | 89.2% |
| %MVC 정규화 | 87.2% | – |
| %MVC + 주파수 특징 2개 (7특징) | 87.5% | 88.1% |
| few-shot 20% 개인보정 | 89.3% | – |
| 학습만 라벨 정제(상위 50%) | 83.6% | 86.9% |

- 개선 시도들은 모두 ±1%p 안이라 발표에서는 86.9% 기준선만 사용한다.
- s3 한 명이 66~75%로 낮고 나머지는 92~96%. 사람 간 편차가 지배적이라 특징 공학으로는 안 오른다.
- few-shot s3 +6.8%p는 시드 의존적이라(재현 편차 큼) 주장에서 뺐다.
- "라벨 정제 후 95%"는 테스트셋까지 정제한 오염된 수치였음 → 철회. 학습만 정제하면 오히려 하락.

## 500Hz·로봇 연결 조건 데이터와의 관계

세션 간 검증(같은 사람의 새 착용, 99.6%)은 상위 폴더 `session_validation.py` + `s1_*_v5~v7.csv`로 별도 수행. 두 수치는 다른 축(처음 보는 사람 vs 같은 사람의 새 착용)이라 같은 표에 놓지 않는다.
