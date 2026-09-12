"""
웹캠 손 추적 → 손가락별 굽힘 정도 확인 (하이브리드 카메라 경로 최소 동작 테스트).

MediaPipe HandLandmarker로 손 관절 21점을 잡고, 손가락 4개(검지/중지/약지/새끼)의
굽힘 비율(0=편 상태, 1=완전히 굽힘)을 계산해서 터미널과 화면에 표시한다.
손이 안 잡히면 "NO HAND" — 이게 나중에 카메라→EMG 전환 신호가 된다.

실행:  python3 hand_track_test.py      (q 키로 종료)
처음 실행 시 macOS가 카메라 권한을 물어보면 허용할 것.
"""

import math
import os
import time
import urllib.request

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")

# MediaPipe 손 관절 인덱스: 손목 0, 각 손가락 (MCP, PIP, DIP, TIP)
FINGERS = {
    "index":  (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring":   (13, 14, 15, 16),
    "pinky":  (17, 18, 19, 20),
}


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print("모델 다운로드 중 (약 8MB)...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("완료:", MODEL_PATH)


def angle(a, b, c):
    """세 점 a-b-c 에서 b를 꼭짓점으로 하는 각도(도)."""
    v1 = (a.x - b.x, a.y - b.y, a.z - b.z)
    v2 = (c.x - b.x, c.y - b.y, c.z - b.z)
    dot = sum(p * q for p, q in zip(v1, v2))
    n1 = math.sqrt(sum(p * p for p in v1)); n2 = math.sqrt(sum(q * q for q in v2))
    if n1 == 0 or n2 == 0:
        return 180.0
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (n1 * n2)))))


def curl_ratio(lm, mcp, pip, dip, tip):
    """PIP 관절 각도로 굽힘 비율 계산. 펴면 ~180도(0.0), 완전히 굽히면 ~40도(1.0)."""
    a = angle(lm[mcp], lm[pip], lm[tip])
    return max(0.0, min(1.0, (180.0 - a) / 140.0))


def main():
    ensure_model()
    options = vision.HandLandmarkerOptions(
        # macOS에서 Metal(GPU) 델리게이트가 "Service is unavailable"로 죽는 경우가 있어 CPU 고정
        base_options=BaseOptions(model_asset_path=MODEL_PATH, delegate=BaseOptions.Delegate.CPU),
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        running_mode=vision.RunningMode.VIDEO,
    )
    landmarker = vision.HandLandmarker.create_from_options(options)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다. 시스템 설정 > 개인정보 보호 > 카메라 에서 터미널 허용 확인")
        return
    print("카메라 열림. 손을 보여주세요. q로 종료.")

    t0 = time.monotonic()
    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        ts_ms = int((time.monotonic() - t0) * 1000)
        result = landmarker.detect_for_video(mp_img, ts_ms)

        h, w = frame.shape[:2]
        if result.hand_landmarks:
            lm = result.hand_landmarks[0]
            curls = {name: curl_ratio(lm, *idx) for name, idx in FINGERS.items()}
            for p in lm:
                cv2.circle(frame, (int(p.x * w), int(p.y * h)), 3, (0, 255, 0), -1)
            y = 30
            for name, c in curls.items():
                bar = "#" * int(c * 20)
                cv2.putText(frame, f"{name:6s} {c:.2f} {bar}", (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y += 25
            cv2.putText(frame, "SOURCE: CAMERA", (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            now = time.monotonic()
            if now - last_print >= 0.5:
                last_print = now
                print("  ".join(f"{k}={v:.2f}" for k, v in curls.items()))
        else:
            cv2.putText(frame, "NO HAND  ->  would switch to EMG", (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("hand track test (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
