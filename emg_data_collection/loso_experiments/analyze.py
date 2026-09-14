import pandas as pd
import glob, os

rows = []
for f in sorted(glob.glob("*.csv")):
    name = os.path.basename(f).replace(".csv", "")
    df = pd.read_csv(f)
    # 앞뒤 1초(100샘플)씩 잘라서 안정 구간만 사용
    core = df["raw"].iloc[100:-100] if len(df) > 250 else df["raw"]
    rows.append({
        "file": name,
        "mean": round(core.mean(), 1),
        "std": round(core.std(), 1),
        "min": core.min(),
        "max": core.max(),
        "n": len(df),
    })

summary = pd.DataFrame(rows)
print(summary.to_string(index=False))
