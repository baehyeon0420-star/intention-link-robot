import pandas as pd
import numpy as np
import glob

WIN = 20      # 200ms
STEP = 10     # 50% 겹침

def extract(sig):
    sig = np.asarray(sig, dtype=float)
    mav = np.mean(np.abs(sig))                    # 평균 절대값
    rms = np.sqrt(np.mean(sig**2))                # RMS
    std = np.std(sig)
    wl  = np.sum(np.abs(np.diff(sig)))            # waveform length
    zc  = np.sum(np.diff(np.sign(sig - sig.mean())) != 0)  # zero crossing
    return [mav, rms, std, wl, zc]

X, y, groups = [], [], []
for f in sorted(glob.glob("*_rest.csv") + glob.glob("*_grip.csv")):
    label = 1 if "grip" in f else 0
    df = pd.read_csv(f)
    subject = df["subject"].iloc[0]
    raw = df["raw"].iloc[100:-100].values   # 과도구간 제거
    for i in range(0, len(raw) - WIN, STEP):
        X.append(extract(raw[i:i+WIN]))
        y.append(label)
        groups.append(subject)

X, y, groups = np.array(X), np.array(y), np.array(groups)
np.save("X.npy", X)
np.save("y.npy", y)
np.save("groups.npy", groups)
print(f"윈도우 {len(X)}개, 특징 {X.shape[1]}개, grip비율 {y.mean():.2f}")
print("사람별 윈도우 수:", {g: int((groups == g).sum()) for g in np.unique(groups)})
