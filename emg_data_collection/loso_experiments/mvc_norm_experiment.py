"""
오늘(9/12) 실험: 기존 s1~s4 rest/grip 데이터만으로, 새 실험 없이 재분석.

1) BASELINE : 기존 train.py와 동일 (raw ADC 윈도우 -> 5특징 -> 전체 pooled StandardScaler)
2) %MVC 정규화: 사람마다 자기 자신의 rest_mean/grip_mean으로 raw 신호를
   0~1 스케일로 먼저 맞춘 뒤(=calibration.py의 rest_baseline/max_contraction과 동일한 개념을
   ML feature extraction 앞단에도 적용) 5특징 추출
3) Few-shot 개인보정: LOSO로 뺀 사람의 윈도우 중 20%만 학습에 섞었을 때
   (zero-shot LOSO) vs (few-shot 20% 보정) 비교

전부 기존 CSV 원본만 사용, 새로 측정한 데이터 없음.
"""
import glob

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.preprocessing import StandardScaler

WIN = 20
STEP = 10
RNG = np.random.RandomState(42)


def extract(sig):
    sig = np.asarray(sig, dtype=float)
    mav = np.mean(np.abs(sig))
    rms = np.sqrt(np.mean(sig**2))
    std = np.std(sig)
    wl = np.sum(np.abs(np.diff(sig)))
    zc = np.sum(np.diff(np.sign(sig - sig.mean())) != 0)
    return [mav, rms, std, wl, zc]


# ── 1. 사람별 raw 신호 로드 + 사람별 rest_mean/grip_mean 계산 ──
subject_raw = {}  # subject -> {"rest": array, "grip": array}
for f in sorted(glob.glob("s*_rest.csv") + glob.glob("s*_grip.csv")):
    label = "grip" if "grip" in f else "rest"
    df = pd.read_csv(f)
    subject = df["subject"].iloc[0]
    raw = df["raw"].iloc[100:-100].values.astype(float)  # 과도구간 제거 (features.py와 동일)
    subject_raw.setdefault(subject, {})[label] = raw

calib = {}
for s, d in subject_raw.items():
    rest_mean = d["rest"].mean()
    grip_mean = d["grip"].mean()
    calib[s] = (rest_mean, grip_mean)
print("사람별 (rest_mean, grip_mean) — calibration.py의 rest_baseline/max_contraction과 동일 개념:")
for s, (r, g) in sorted(calib.items()):
    print(f"  {s}: rest={r:.1f}  grip={g:.1f}  ratio(grip/rest 대비 폭)={g - r:.1f}")


def build_dataset(normalize: bool):
    X, y, groups = [], [], []
    for s, d in subject_raw.items():
        rest_mean, grip_mean = calib[s]
        span = grip_mean - rest_mean
        for label_name, raw in d.items():
            sig = raw
            if normalize:
                sig = (raw - rest_mean) / span  # %MVC 스타일 정규화 (0=rest, 1=grip 평균)
            label = 1 if label_name == "grip" else 0
            for i in range(0, len(sig) - WIN, STEP):
                X.append(extract(sig[i : i + WIN]))
                y.append(label)
                groups.append(s)
    return np.array(X), np.array(y), np.array(groups)


def loso(X, y, groups, Model, kwargs):
    accs = {}
    for test_subject in np.unique(groups):
        tr = groups != test_subject
        te = groups == test_subject
        sc = StandardScaler().fit(X[tr])
        m = Model(**kwargs)
        m.fit(sc.transform(X[tr]), y[tr])
        accs[test_subject] = accuracy_score(y[te], m.predict(sc.transform(X[te])))
    return accs


def few_shot(X, y, groups, Model, kwargs, calib_frac=0.2):
    """LOSO로 뺀 사람의 윈도우 중 calib_frac만 학습에 추가 (zero-shot vs few-shot 비교)."""
    accs_zero, accs_few = {}, {}
    for test_subject in np.unique(groups):
        tr = groups != test_subject
        te_idx = np.where(groups == test_subject)[0]
        RNG.shuffle(te_idx)
        n_calib = max(1, int(len(te_idx) * calib_frac))
        calib_idx = te_idx[:n_calib]
        eval_idx = te_idx[n_calib:]

        # zero-shot: 다른 사람 데이터만
        sc0 = StandardScaler().fit(X[tr])
        m0 = Model(**kwargs)
        m0.fit(sc0.transform(X[tr]), y[tr])
        accs_zero[test_subject] = accuracy_score(y[eval_idx], m0.predict(sc0.transform(X[eval_idx])))

        # few-shot: 다른 사람 + 이 사람 20%
        tr_few_idx = np.concatenate([np.where(tr)[0], calib_idx])
        sc1 = StandardScaler().fit(X[tr_few_idx])
        m1 = Model(**kwargs)
        m1.fit(sc1.transform(X[tr_few_idx]), y[tr_few_idx])
        accs_few[test_subject] = accuracy_score(y[eval_idx], m1.predict(sc1.transform(X[eval_idx])))
    return accs_zero, accs_few


MODELS = [
    ("LogisticRegression", LogisticRegression, dict(max_iter=1000)),
    ("RandomForest", RandomForestClassifier, dict(n_estimators=100, random_state=42)),
]

for tag, normalize in [("BASELINE (raw, 기존 방식)", False), ("%MVC 정규화 적용", True)]:
    print("\n" + "=" * 70)
    print(f"[{tag}]")
    print("=" * 70)
    X, y, groups = build_dataset(normalize)
    for name, Model, kwargs in MODELS:
        accs = loso(X, y, groups, Model, kwargs)
        for s, a in sorted(accs.items()):
            print(f"  {name:20s} test={s}: acc={a:.3f}")
        print(f"  -> {name} LOSO 평균: {np.mean(list(accs.values())):.3f}\n")

# ── few-shot 개인보정: %MVC 정규화 데이터 기준으로 ──
print("=" * 70)
print("[Few-shot 개인보정] (%MVC 정규화 데이터, 대상자 윈도우의 20%만 학습에 추가)")
print("=" * 70)
X, y, groups = build_dataset(normalize=True)
for name, Model, kwargs in MODELS:
    zero, few = few_shot(X, y, groups, Model, kwargs, calib_frac=0.2)
    print(f"  [{name}]")
    for s in sorted(zero):
        print(f"    {s}: zero-shot={zero[s]:.3f}  ->  few-shot(20%)={few[s]:.3f}  (Δ={few[s]-zero[s]:+.3f})")
    print(f"    평균: zero-shot={np.mean(list(zero.values())):.3f}  ->  few-shot={np.mean(list(few.values())):.3f}\n")

# ── 추가 실험: 주파수영역 특징(MNF/스펙트럼 에너지) 2개를 더해서 도움이 되는지 확인 ──
def extract_with_freq(sig):
    base = extract(sig)
    sig = np.asarray(sig, dtype=float)
    fft_vals = np.abs(np.fft.rfft(sig - sig.mean()))
    freqs = np.fft.rfftfreq(len(sig), d=1 / 100.0)  # 100Hz 샘플링
    power = fft_vals ** 2
    total_power = power.sum() + 1e-9
    mnf = (freqs * power).sum() / total_power  # mean frequency
    return base + [mnf, total_power]


def build_dataset_freq(normalize: bool):
    X, y, groups = [], [], []
    for s, d in subject_raw.items():
        rest_mean, grip_mean = calib[s]
        span = grip_mean - rest_mean
        for label_name, raw in d.items():
            sig = (raw - rest_mean) / span if normalize else raw
            label = 1 if label_name == "grip" else 0
            for i in range(0, len(sig) - WIN, STEP):
                X.append(extract_with_freq(sig[i : i + WIN]))
                y.append(label)
                groups.append(s)
    return np.array(X), np.array(y), np.array(groups)


print("\n" + "=" * 70)
print("[추가] %MVC 정규화 + 주파수특징(MNF, 스펙트럼 에너지) 2개 추가 (총 7특징)")
print("=" * 70)
Xf, yf, gf = build_dataset_freq(normalize=True)
for name, Model, kwargs in MODELS:
    accs = loso(Xf, yf, gf, Model, kwargs)
    for s, a in sorted(accs.items()):
        print(f"  {name:20s} test={s}: acc={a:.3f}")
    print(f"  -> {name} LOSO 평균: {np.mean(list(accs.values())):.3f}\n")
