"""
웹캠 손 추적 → AmazingHand 손가락별 제어 (하이브리드 카메라 경로 실물 테스트).

카메라에서 손가락 4개 굽힘값(0=폄, 1=굽힘)을 뽑아 로봇 손가락 A/B/C/D에 전달한다.
손이 안 보이면(NO HAND) 마지막 자세를 유지한다 — 이 자리가 나중에 EMG로 넘어가는 지점.

서보 명령은 (1) 굽힘값이 min_delta 이상 변한 손가락만, (2) 초당 send_hz 회 이하로만 보낸다.
(이전에 매 루프마다 8개 서보에 16회씩 쓰다가 버스가 포화돼 손이 굳었던 버그의 재발 방지)

실행:
  python3 hand_to_robot.py --hand-port /dev/cu.usbmodem5B790178941
  (q 키로 종료 → 토크 해제)

손가락 매핑이 틀리면(예: 검지를 굽혔는데 다른 손가락이 움직임) --map 으로 순서를 바꾼다:
  --map index:A,middle:B,ring:C,pinky:D   (기본값)
"""

import argparse
import math
import os
import time
import urllib.request

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

from emg_pipeline.amazinghand_driver import AmazingHandDriver

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "emg_data_collection", "hand_landmarker.task")

FINGERS = {
    "index":  (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring":   (13, 14, 15, 16),
    "pinky":  (17, 18, 19, 20),
}


def parse_args():
    p = argparse.ArgumentParser(description="웹캠 손 추적 → AmazingHand 손가락별 제어")
    p.add_argument("--hand-port", required=True, help="AmazingHand USB-TTL 포트")
    p.add_argument("--map", default="index:A,middle:B,ring:C,pinky:D",
                   help="카메라 손가락 → 로봇 손가락(A~D) 매핑")
    p.add_argument("--send-hz", type=float, default=10.0, help="서보 명령 최대 전송 빈도(초당)")
    p.add_argument("--min-delta", type=float, default=0.08, help="이 이상 변해야 그 손가락에 명령 전송")
    return p.parse_args()


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print("모델 다운로드 중...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


def angle(a, b, c):
    v1 = (a.x - b.x, a.y - b.y, a.z - b.z); v2 = (c.x - b.x, c.y - b.y, c.z - b.z)
    dot = sum(p * q for p, q in zip(v1, v2))
    n1 = math.sqrt(sum(p * p for p in v1)); n2 = math.sqrt(sum(q * q for q in v2))
    if n1 == 0 or n2 == 0:
        return 180.0
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (n1 * n2)))))


def curl_ratio(lm, mcp, pip, dip, tip):
    return max(0.0, min(1.0, (180.0 - angle(lm[mcp], lm[pip], lm[tip])) / 140.0))


def main():
    args = parse_args()
    mapping = dict(kv.split(":") for kv in args.map.split(","))
    ensure_model()

    landmarker = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH, delegate=BaseOptions.Delegate.CPU),
        num_hands=1, min_hand_detection_confidence=0.5, min_tracking_confidence=0.5,
        running_mode=vision.RunningMode.VIDEO))

    hand = AmazingHandDriver(port=args.hand_port)
    print(f"[robot] AmazingHand 연결: {args.hand_port}")
    hand.release()  # 시작은 편 상태

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다"); hand.shutdown(); return
    print("손을 보여주세요. q로 종료.")

    t0 = time.monotonic(); last_send = 0.0; send_interval = 1.0 / args.send_hz
    sent_count = 0; t_stat = time.monotonic()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                              int((time.monotonic() - t0) * 1000))
            h, w = frame.shape[:2]
            if res.hand_landmarks:
                lm = res.hand_landmarks[0]
                curls = {name: curl_ratio(lm, *idx) for name, idx in FINGERS.items()}
                for p in lm:
                    cv2.circle(frame, (int(p.x * w), int(p.y * h)), 3, (0, 255, 0), -1)
                y = 30
                for name, c in curls.items():
                    cv2.putText(frame, f"{name:6s} {c:.2f} {'#' * int(c * 20)}", (10, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2); y += 25
                cv2.putText(frame, "SOURCE: CAMERA", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                now = time.monotonic()
                if now - last_send >= send_interval:
                    last_send = now
                    ratios = {mapping[name]: c for name, c in curls.items() if name in mapping}
                    n = hand.set_fingers(ratios, min_delta=args.min_delta)
                    sent_count += n
            else:
                cv2.putText(frame, "NO HAND -> hold last pose (EMG would take over)", (10, h - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if time.monotonic() - t_stat >= 2.0:
                print(f"[stat] 최근 2초 서보 쓰기 {sent_count}회 (손가락 단위)")
                sent_count = 0; t_stat = time.monotonic()

            cv2.imshow("hand -> robot (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release(); cv2.destroyAllWindows()
        hand.shutdown()
        print("[robot] 토크 해제, 종료")


if __name__ == "__main__":
    main()
