"""
세션 간 일반화 검증 (2026-09-12).

같은 사람(s1)이 같은 날, 같은 펌웨어(500Hz)로 전극을 떼었다 붙여가며 받은
세 세션(v5/v6/v7)으로 "한 세션으로만 학습한 모델이 다른 세션에서도 되는가"를 본다.

  v5: 로봇 USB 분리, rest 잡음 std 5.9
  v6: 로봇 USB 연결, rest 잡음 std 19.5  (전극 재부착)
  v7: 로봇 USB 연결, rest 잡음 std 19.5  (전극 재부착)

결과 요약 (docs/2026-09-12_작업정리.md 참고):
  - v5로 학습 -> v6/v7 평가: 0.643 (전부 grip으로 오판, 사실상 작동 불능)
  - 나머지 방향: 0.985~1.000
  - 두 세션 섞어 학습 -> 나머지 평가(LOSO-session): LogReg 0.996 / RF 0.970
  => 조용한 조건으로만 학습하면 시끄러운 조건에서 붕괴. 여러 조건을 섞어 학습해야 함.

실행: python3 session_validation.py  (이 폴더에서)
"""
import itertools

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score
from sklearn.preprocessing import StandardScaler

WIN, STEP = 20, 10
SESSIONS = ["v5", "v6", "v7"]


def extract(s):
    s = np.asarray(s, float)
    return [
        np.mean(np.abs(s)),
        np.sqrt(np.mean(s**2)),
        np.std(s),
        np.sum(np.abs(np.diff(s))),
        np.sum(np.diff(np.sign(s - s.mean())) != 0),
    ]


def load(f):
    v = pd.to_numeric(pd.read_csv(f)["raw"], errors="coerce").dropna()
    return v[(v >= 0) & (v <= 4095)].values


def windows(raw):
    return [extract(raw[i : i + WIN]) for i in range(0, len(raw) - WIN, STEP)]


def dataset(v):
    r, g = load(f"s1_rest_{v}.csv"), load(f"s1_grip_{v}.csv")
    X = windows(r) + windows(g)
    y = [0] * len(windows(r)) + [1] * len(windows(g))
    return np.array(X), np.array(y)


def fit_eval(Xtr, ytr, Xte, yte, Model=RandomForestClassifier, kw=None):
    kw = kw or dict(n_estimators=100, random_state=42)
    sc = StandardScaler().fit(Xtr)
    m = Model(**kw).fit(sc.transform(Xtr), ytr)
    p = m.predict(sc.transform(Xte))
    return accuracy_score(yte, p), p.mean()


def main():
    D = {v: dataset(v) for v in SESSIONS}

    print("■ 1. 세션별 신호 상태")
    for v in SESSIONS:
        r, g = load(f"s1_rest_{v}.csv"), load(f"s1_grip_{v}.csv")
        print(f"   {v}: rest={r.mean():5.0f} (std {r.std():4.1f})  grip={g.mean():5.0f}  차이={g.mean()-r.mean():4.0f}")

    print("\n■ 2. 한 세션 학습 → 다른 세션 평가  (정확도 / grip으로 예측한 비율)")
    for a, b in itertools.permutations(SESSIONS, 2):
        acc, pg = fit_eval(*D[a], *D[b])
        flag = "  ← 전부 grip으로 오판" if pg > 0.99 else ""
        print(f"   {a}→{b}: {acc:.3f}  (grip 예측 {pg*100:5.1f}%){flag}")

    print("\n■ 3. 두 세션 섞어 학습 → 학습에 없던 세션 평가 (Leave-One-Session-Out)")
    rows = []
    for hold in SESSIONS:
        tr = [v for v in SESSIONS if v != hold]
        Xtr = np.vstack([D[v][0] for v in tr])
        ytr = np.concatenate([D[v][1] for v in tr])
        lr, _ = fit_eval(Xtr, ytr, *D[hold], Model=LogisticRegression, kw=dict(max_iter=1000))
        rf, _ = fit_eval(Xtr, ytr, *D[hold])
        rows.append((lr, rf))
        print(f"   {'+'.join(tr)} → {hold}:  LogReg={lr:.3f}  RF={rf:.3f}")
    r = np.array(rows)
    print(f"   평균: LogReg={r[:,0].mean():.3f}  RF={r[:,1].mean():.3f}")


if __name__ == "__main__":
    main()
