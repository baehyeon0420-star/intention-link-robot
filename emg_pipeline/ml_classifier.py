"""
train.py가 학습해서 저장한 final_model.npz(StandardScaler + LogisticRegression)를
그대로 불러와 실시간 추론에 쓰는 분류기.

intention-link(Unity)의 EMGMLClassifier.cs를 이식한 것 — 그쪽은 계수를 코드에
하드코딩했지만, 여기서는 원본 학습 산출물(final_model.npz)을 직접 로드해서
"진짜 소스"와 항상 일치하도록 함.

특징 순서(features.py extract()와 동일): [mav, rms, std, wl, zc]
"""

import math

import numpy as np

FEATURE_NAMES = ["mav", "rms", "std", "wl", "zc"]


class EMGMLClassifier:
    def __init__(self, model_path="final_model.npz", decision_threshold=0.5):
        data = np.load(model_path)
        self.mean = data["mean"]
        self.scale = data["scale"]
        self.coef = data["coef"]
        self.intercept = float(data["intercept"])
        self.decision_threshold = decision_threshold

    @staticmethod
    def extract_features(window):
        """features.py의 extract()와 동일한 공식."""
        sig = np.asarray(window, dtype=float)
        mav = np.mean(np.abs(sig))
        rms = np.sqrt(np.mean(sig ** 2))
        std = np.std(sig)
        wl = np.sum(np.abs(np.diff(sig)))
        zc = np.sum(np.diff(np.sign(sig - sig.mean())) != 0)
        return np.array([mav, rms, std, wl, zc], dtype=float)

    def predict_proba(self, features):
        scaled = (features - self.mean) / self.scale
        z = self.intercept + float(np.dot(self.coef, scaled))
        return 1.0 / (1.0 + math.exp(-z))

    def classify(self, window):
        """window(raw 샘플 리스트) -> (state, probability). state는 "GRIP" 또는 "REST"."""
        features = self.extract_features(window)
        proba = self.predict_proba(features)
        state = "GRIP" if proba >= self.decision_threshold else "REST"
        return state, proba
