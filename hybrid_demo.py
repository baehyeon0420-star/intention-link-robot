"""
하이브리드 데모: 손이 카메라에 보이면 카메라로, 안 보이면 EMG로 로봇손 제어.

  카메라 보임  → 손가락 4개 굽힘값 → 로봇 손가락별 제어 (set_fingers)
  카메라 안 보임 → EMG 임계값 상태(REST/LIGHT/STRONG/GRIP) → 로봇 전체 쥐기 정도

전환은 떨림 방지를 위해 몇 프레임 연속으로 확인된 뒤에만 일어난다.
서보 명령은 변화가 있을 때만, 초당 send_hz 회 이하로만 보낸다.

실행 순서:
  1) 전극 부착, ESP32·AmazingHand 연결
  2) python3 hybrid_demo.py --port /dev/cu.usbserial-110 --hand-port /dev/cu.usbmodem5B790178941
  3) 터미널 안내대로 EMG 캘리브레이션 (힘 빼기 3초 → 세게 쥐기 3초)
  4) 카메라 창이 뜨면 손 보여주기 → 손 숨기고 주먹 쥐기 → 다시 보여주기
  5) q 종료 (토크 해제)
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
from emg_pipeline.calibration import auto_calibrate
from emg_pipeline.serial_reader import EMGSerialReader

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "emg_data_collection", "hand_landmarker.task")
FINGERS = {"index": (5, 6, 7, 8), "middle": (9, 10, 11, 12), "ring": (13, 14, 15, 16), "pinky": (17, 18, 19, 20)}
# EMG 상태 → 전체 손 쥐기 비율 (검증된 3자세: 펴짐 / 중간 / 접힘)
EMG_RATIO = {"REST": 0.0, "LIGHT": 0.5, "STRONG": 1.0, "GRIP": 1.0}


def parse_args():
    p = argparse.ArgumentParser(description="카메라↔EMG 하이브리드 로봇손 제어")
    p.add_argument("--port", required=True, help="ESP32(EMG) 시리얼 포트")
    p.add_argument("--hand-port", required=True, help="AmazingHand USB-TTL 포트")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--map", default="thumb:A,index:B,middle:C,ring:D",
                   help="카메라 손가락 → 로봇 손가락. 로봇은 엄지·검지·중지·약지 4개(새끼 없음)")
    p.add_argument("--thumb-gain", type=float, default=1.0, help="엄지 굽힘 감도 (덜 굽으면 1.3~1.5로)")
    p.add_argument("--send-hz", type=float, default=10.0)
    p.add_argument("--min-delta", type=float, default=0.08)
    p.add_argument("--lost-frames", type=int, default=8, help="이 프레임 수만큼 손이 안 보이면 EMG로 전환 (~0.3초)")
    p.add_argument("--found-frames", type=int, default=3, help="이 프레임 수만큼 손이 보이면 카메라로 복귀")
    p.add_argument("--calibrate-seconds", type=float, default=3.0)
    return p.parse_args()


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print("손 모델 다운로드 중..."); urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


def angle(a, b, c):
    v1 = (a.x - b.x, a.y - b.y, a.z - b.z); v2 = (c.x - b.x, c.y - b.y, c.z - b.z)
    dot = sum(p * q for p, q in zip(v1, v2))
    n1 = math.sqrt(sum(p * p for p in v1)); n2 = math.sqrt(sum(q * q for q in v2))
    return 180.0 if n1 == 0 or n2 == 0 else math.degrees(math.acos(max(-1.0, min(1.0, dot / (n1 * n2)))))


def curl_ratio(lm, mcp, pip, dip, tip):
    return max(0.0, min(1.0, (180.0 - angle(lm[mcp], lm[pip], lm[tip])) / 140.0))


def thumb_curl(lm, gain=1.0):
    """엄지는 다른 손가락과 관절 구조가 달라 IP(3)·MCP(2) 두 관절 굽힘을 합산. 범위가 좁아 분모를 작게."""
    a_ip = angle(lm[2], lm[3], lm[4])    # 펴면 ~180
    a_mcp = angle(lm[1], lm[2], lm[3])   # 펴면 ~170
    return max(0.0, min(1.0, ((180.0 - a_ip) + (170.0 - a_mcp)) / 100.0 * gain))


def all_curls(lm, thumb_gain=1.0):
    c = {"thumb": thumb_curl(lm, thumb_gain)}
    c.update({name: curl_ratio(lm, *idx) for name, idx in FINGERS.items()})
    return c


def main():
    args = parse_args()
    mapping = dict(kv.split(":") for kv in args.map.split(","))
    ensure_model()

    # 1) EMG 먼저 열고 캘리브레이션 (main.py와 같은 순서: ESP32 → 잠깐 대기 → AmazingHand)
    reader = EMGSerialReader(port=args.port, baud_rate=args.baud)
    reader.start()
    print(f"[emg] 연결: {args.port}")
    time.sleep(0.5)
    rb, mc = auto_calibrate(reader, seconds=args.calibrate_seconds)
    reader.rest_baseline, reader.max_contraction = rb, mc

    # 2) 로봇
    hand = AmazingHandDriver(port=args.hand_port)
    print(f"[robot] 연결: {args.hand_port}")
    hand.release()

    # 3) 카메라
    landmarker = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH, delegate=BaseOptions.Delegate.CPU),
        num_hands=1, min_hand_detection_confidence=0.5, min_tracking_confidence=0.5,
        running_mode=vision.RunningMode.VIDEO))
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다"); hand.shutdown(); reader.stop(); return
    print("\n손을 보여주면 CAMERA, 숨기면 EMG. q로 종료.\n")

    source = "EMG"           # 시작은 EMG (손이 아직 안 보이니까)
    seen, lost = 0, 0
    t0 = time.monotonic(); last_send = 0.0; interval = 1.0 / args.send_hz
    sent = 0; t_stat = time.monotonic(); switches = 0
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
            visible = bool(res.hand_landmarks)

            # ── 전환 판단 (연속 프레임으로 떨림 방지) ──
            if visible:
                seen += 1; lost = 0
                if source == "EMG" and seen >= args.found_frames:
                    source = "CAMERA"; switches += 1; print("[switch] → CAMERA")
            else:
                lost += 1; seen = 0
                if source == "CAMERA" and lost >= args.lost_frames:
                    source = "EMG"; switches += 1; print("[switch] → EMG")

            # ── 명령 생성 ──
            now = time.monotonic()
            ratios = None
            if source == "CAMERA" and visible:
                lm = res.hand_landmarks[0]
                curls = all_curls(lm, args.thumb_gain)
                ratios = {mapping[n]: c for n, c in curls.items() if n in mapping}
                for p in lm:
                    cv2.circle(frame, (int(p.x * w), int(p.y * h)), 3, (0, 255, 0), -1)
                y = 30
                for n, c in curls.items():
                    cv2.putText(frame, f"{n:6s} {c:.2f} {'#' * int(c * 20)}", (10, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2); y += 25
            elif source == "EMG":
                r = EMG_RATIO.get(reader.current_state, 0.0)
                ratios = {"A": r, "B": r, "C": r, "D": r}
                cv2.putText(frame, f"EMG state={reader.current_state}  norm={reader.current_normalized:.2f}  raw={reader.current_value}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            if ratios is not None and now - last_send >= interval:
                last_send = now
                sent += hand.set_fingers(ratios, min_delta=args.min_delta)

            color = (0, 255, 0) if source == "CAMERA" else (0, 200, 255)
            cv2.putText(frame, f"SOURCE: {source}", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)

            if now - t_stat >= 2.0:
                print(f"[stat] source={source:6s} emg={reader.current_state:6s} 서보쓰기 {sent}회/2초 전환 {switches}회")
                sent = 0; t_stat = now

            cv2.imshow("hybrid: camera <-> EMG (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release(); cv2.destroyAllWindows()
        hand.shutdown(); reader.stop()
        print("[종료] 토크 해제")


if __name__ == "__main__":
    main()
