"""
압력센서 없이, 기존 s1~s4 데이터만으로 돌리는 심화 검증 3종.

실험1. 학습만 정제 / 테스트는 원본  (정제가 "시험을 쉽게 만든 것"이 아님을 증명)
실험2. 정제 비율 민감도            (50%라는 기준이 자의적이지 않음을 증명)
실험3. few-shot 개인보정 곡선       ("신규 사용자 데이터가 얼마나 필요한가"를 정량화)
"""
import glob
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score
from sklearn.preprocessing import StandardScaler

WIN, STEP = 20, 10
RNG = np.random.RandomState(42)

def extract(sig):
    sig = np.asarray(sig, dtype=float)
    return [np.mean(np.abs(sig)), np.sqrt(np.mean(sig**2)), np.std(sig),
            np.sum(np.abs(np.diff(sig))), np.sum(np.diff(np.sign(sig - sig.mean())) != 0)]

subject_raw = {}
for f in sorted(glob.glob("s*_rest.csv") + glob.glob("s*_grip.csv")):
    label = "grip" if "grip" in f else "rest"
    df = pd.read_csv(f)
    s = df["subject"].iloc[0]
    subject_raw.setdefault(s, {})[label] = df["raw"].iloc[100:-100].values.astype(float)

SUBJECTS = sorted(subject_raw)

def windows_of(sig):
    return [sig[i:i+WIN] for i in range(0, len(sig)-WIN, STEP)]

def build(subject, keep_ratio):
    """keep_ratio=1.0이면 원본 그대로. 1.0 미만이면 grip 윈도우 중 진폭 상위 keep_ratio만 남김."""
    d = subject_raw[subject]
    X, y = [], []
    for w in windows_of(d["rest"]):
        X.append(extract(w)); y.append(0)
    gw = windows_of(d["grip"])
    if keep_ratio < 1.0:
        means = np.array([w.mean() for w in gw])
        thresh = np.quantile(means, 1 - keep_ratio)
        gw = [w for w, m in zip(gw, means) if m >= thresh]
    for w in gw:
        X.append(extract(w)); y.append(1)
    return np.array(X), np.array(y)

MODELS = [("LogReg", LogisticRegression, dict(max_iter=1000)),
          ("RF", RandomForestClassifier, dict(n_estimators=100, random_state=42))]

def loso_traintest(train_keep, test_keep):
    """train_keep 비율로 정제한 데이터로 학습, test_keep 비율 데이터로 평가."""
    out = {}
    for name, Model, kw in MODELS:
        accs = {}
        for test_s in SUBJECTS:
            Xtr, ytr = [], []
            for s in SUBJECTS:
                if s == test_s: continue
                Xs, ys = build(s, train_keep)
                Xtr.append(Xs); ytr.append(ys)
            Xtr, ytr = np.vstack(Xtr), np.concatenate(ytr)
            Xte, yte = build(test_s, test_keep)
            sc = StandardScaler().fit(Xtr)
            m = Model(**kw); m.fit(sc.transform(Xtr), ytr)
            accs[test_s] = accuracy_score(yte, m.predict(sc.transform(Xte)))
        out[name] = accs
    return out

print("=" * 72)
print("[실험1] 테스트는 항상 원본(정제 안 함). 학습 데이터만 정제했을 때의 효과")
print("=" * 72)
for label, tk in [("학습도 원본 (기존 보고서 방식)", 1.0), ("학습만 정제(상위50%)", 0.5)]:
    res = loso_traintest(train_keep=tk, test_keep=1.0)
    for name in res:
        accs = res[name]
        detail = "  ".join(f"{s}={accs[s]:.3f}" for s in SUBJECTS)
        print(f"  {label:28s} {name:7s} 평균={np.mean(list(accs.values())):.3f}   ({detail})")
    print()

print("=" * 72)
print("[실험2] 정제 비율 민감도 (학습만 정제, 테스트는 원본)")
print("=" * 72)
print(f"  {'학습 유지비율':>12s}  {'LogReg':>8s}  {'RF':>8s}   {'s3(LogReg)':>10s}")
for kr in [1.0, 0.7, 0.6, 0.5, 0.4, 0.3]:
    res = loso_traintest(train_keep=kr, test_keep=1.0)
    lg = np.mean(list(res["LogReg"].values())); rf = np.mean(list(res["RF"].values()))
    print(f"  {kr*100:10.0f}%  {lg:8.3f}  {rf:8.3f}   {res['LogReg']['s3']:10.3f}")

print()
print("=" * 72)
print("[실험3] few-shot 개인보정 곡선 (학습:다른3명 원본 + 대상자 일부 / 평가:대상자 나머지)")
print("=" * 72)
data = {s: build(s, 1.0) for s in SUBJECTS}
print(f"  {'보정데이터':>8s}  " + "  ".join(f"{s:>7s}" for s in SUBJECTS) + "     평균")
for frac in [0.0, 0.05, 0.10, 0.20, 0.30, 0.40]:
    accs = {}
    for test_s in SUBJECTS:
        Xtr = np.vstack([data[s][0] for s in SUBJECTS if s != test_s])
        ytr = np.concatenate([data[s][1] for s in SUBJECTS if s != test_s])
        Xte_all, yte_all = data[test_s]
        idx = np.arange(len(Xte_all)); RNG.shuffle(idx)
        n_cal = int(len(idx) * frac)
        cal, ev = idx[:n_cal], idx[n_cal:]
        if n_cal > 0:
            Xtr = np.vstack([Xtr, Xte_all[cal]]); ytr = np.concatenate([ytr, yte_all[cal]])
        sc = StandardScaler().fit(Xtr)
        m = RandomForestClassifier(n_estimators=100, random_state=42).fit(sc.transform(Xtr), ytr)
        accs[test_s] = accuracy_score(yte_all[ev], m.predict(sc.transform(Xte_all[ev])))
    row = "  ".join(f"{accs[s]:7.3f}" for s in SUBJECTS)
    print(f"  {frac*100:7.0f}%  {row}   {np.mean(list(accs.values())):7.3f}")
