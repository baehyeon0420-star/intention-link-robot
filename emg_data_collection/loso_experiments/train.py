import numpy as np
from sklearn.model_selection import train_test_split, LeaveOneGroupOut
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, accuracy_score

X = np.load("X.npy")
y = np.load("y.npy")
groups = np.load("groups.npy")

FEATURE_NAMES = ["mav", "rms", "std", "wl", "zc"]

# ── 1) 순진하게 섞어서 나눈 경우 (뻥튀기된 정확도) ──
print("=" * 60)
print("[1] 랜덤 split (같은 사람 윈도우가 train/test에 섞임 — 참고용)")
print("=" * 60)
Xtr, Xte, ytr, yte = train_test_split(X, y, test_size=0.25, stratify=y, random_state=42)
scaler = StandardScaler().fit(Xtr)
clf = RandomForestClassifier(n_estimators=100, random_state=42)
clf.fit(scaler.transform(Xtr), ytr)
acc_naive = accuracy_score(yte, clf.predict(scaler.transform(Xte)))
print(f"RandomForest 정확도: {acc_naive:.3f}")

# ── 2) LOSO: 한 사람을 통째로 빼고 학습 → 그 사람으로 테스트 ──
print()
print("=" * 60)
print("[2] LOSO (Leave-One-Subject-Out) — 새로운 사람에 대한 실제 일반화 성능")
print("=" * 60)

logo = LeaveOneGroupOut()
for name, Model, kwargs in [
    ("RandomForest", RandomForestClassifier, dict(n_estimators=100, random_state=42)),
    ("LogisticRegression", LogisticRegression, dict(max_iter=1000)),
]:
    accs = []
    for test_subject in np.unique(groups):
        train_idx = groups != test_subject
        test_idx = groups == test_subject
        sc = StandardScaler().fit(X[train_idx])
        m = Model(**kwargs)
        m.fit(sc.transform(X[train_idx]), y[train_idx])
        acc = accuracy_score(y[test_idx], m.predict(sc.transform(X[test_idx])))
        accs.append(acc)
        print(f"  {name:20s} test={test_subject}: acc={acc:.3f}")
    print(f"  → {name} LOSO 평균 정확도: {np.mean(accs):.3f}\n")

# ── 3) Unity 이식용 최종 모델: 전체 데이터로 LogisticRegression 학습 ──
print("=" * 60)
print("[3] Unity 이식용 최종 모델 (전체 4명 데이터로 학습한 LogisticRegression)")
print("=" * 60)

final_scaler = StandardScaler().fit(X)
Xs = final_scaler.transform(X)
final_clf = LogisticRegression(max_iter=1000)
final_clf.fit(Xs, y)

print("StandardScaler mean:", final_scaler.mean_.tolist())
print("StandardScaler scale:", final_scaler.scale_.tolist())
print("LogisticRegression coef:", final_clf.coef_[0].tolist())
print("LogisticRegression intercept:", final_clf.intercept_[0])
print("Feature order:", FEATURE_NAMES)

np.savez("final_model.npz",
         mean=final_scaler.mean_, scale=final_scaler.scale_,
         coef=final_clf.coef_[0], intercept=final_clf.intercept_[0])
print("\nfinal_model.npz 저장 완료")
