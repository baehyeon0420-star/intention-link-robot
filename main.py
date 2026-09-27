"""
Unity 없이 파이썬만으로 돌아가는 intention-link 실시간 EMG -> 로봇팔 제어 루프.

구성 (Unity 버전과의 대응):
  EMGSerialReader.cs      -> emg_pipeline/serial_reader.py
  EMGMLClassifier.cs      -> emg_pipeline/ml_classifier.py
  RobotCommandMapper.cs   -> emg_pipeline/command_mapper.py (map_to_robot_command)
  EMGStateManager.cs      -> emg_pipeline/command_mapper.py (ContractionPatternDetector)
  RobotArmController.cs   -> emg_pipeline/robot_controller.py

실제 로봇 명령은 2상태(GripLatch: STRONG/GRIP→GripClose, REST→Release, LIGHT→이전 명령 유지)로
내려진다. 4단계 threshold 상태와 ML 분류기(final_model.npz)는 디버그/비교용으로 같이 출력된다.
(2026-09-22) 보정 기준을 "세게 쥐기"에서 "편하게 쥐기"(--ref-contraction)로 바꿈.
"""

import argparse
import time

from emg_pipeline.command_mapper import GripLatch
from emg_pipeline.ml_classifier import EMGMLClassifier
from emg_pipeline.robot_controller import RobotArmController
from emg_pipeline.serial_reader import EMGSerialReader


def parse_args():
    p = argparse.ArgumentParser(description="intention-link 실시간 EMG 제어 (Unity 미사용)")
    p.add_argument("--port", required=True, help="ESP32(EMG) 시리얼 포트, 예: /dev/cu.usbserial-1110")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--rest-baseline", type=int, default=18, help="힘 뺀 상태 raw 중앙값. 임시 기본값 — 반드시 --calibrate로 재보정")
    p.add_argument("--ref-contraction", type=int, default=400,
                   help="물건 집듯이 편하게 쥔 raw 중앙값 (norm=1.0 기준). 임시 기본값 — 반드시 --calibrate로 재보정")
    p.add_argument("--max-contraction", type=int, default=None,
                   help="(구버전) --ref-contraction 과 같음. 쓰면 경고 후 ref로 매핑")
    p.add_argument("--threshold-strong", type=float, default=0.65,
                   help="쥐기(STRONG) 진입 기준, 편한 쥐기 대비 비율. 기본 0.65 (analysis/compare_thresholds.py 로 재조정)")
    p.add_argument("--threshold-light", type=float, default=0.2, help="LIGHT 진입 기준 (LIGHT 는 이전 명령 유지). 기본 0.2")
    p.add_argument("--release-ratio", type=float, default=0.43,
                   help="상태가 내려갈 때 기준 = threshold x 이 값 (히스테리시스). 기본 0.43 → 쥐기는 0.28 까지 유지")
    p.add_argument(
        "--calibrate",
        action="store_true",
        help="시작할 때 '힘 빼기 3초' → '편하게 쥐기 3초'를 측정해서 rest/ref를 자동으로 잡고 calibration.json에 저장. "
        "전극 상태가 매번 달라지니 세션마다 쓰는 걸 추천.",
    )
    p.add_argument("--use-saved", action="store_true",
                   help="calibration.json에 저장된 보정값을 읽어서 씀 (--calibrate 없이). 전극을 다시 붙였다면 쓰지 말 것")
    p.add_argument("--calibrate-seconds", type=float, default=3.0, help="캘리브레이션 각 단계 측정 시간(초)")
    p.add_argument(
        "--smoothing",
        type=float,
        default=0.02,
        help="EMG 신호 이동평균 강도(0~1). 작을수록 더 부드럽지만 반응이 느려짐. "
        "기본 0.02 (500Hz에서 시간상수 약 100ms)",
    )
    p.add_argument("--model", default="final_model.npz")
    p.add_argument("--interval", type=float, default=0.05, help="제어 루프 주기(초), 기본 50ms")
    p.add_argument(
        "--print-interval",
        type=float,
        default=0.2,
        help="화면 상태 출력 주기(초), 기본 200ms. 제어 반응 속도(--interval)와는 별개 — "
        "값을 줄이면 더 자주 찍히지만 화면이 정신없어짐.",
    )
    p.add_argument(
        "--hand-port",
        default=None,
        help="AmazingHand용 USB-TTL 포트 (예: /dev/cu.usbmodem5B790178941). "
        "ESP32 포트와는 다른 별도 USB 장치. 안 주면 콘솔 출력만 하고 실제 손은 안 움직임.",
    )
    return p.parse_args()


def main():
    args = parse_args()
    if args.max_contraction is not None:
        print("[main] 경고: --max-contraction 은 구버전 옵션입니다. --ref-contraction 으로 취급합니다.")
        args.ref_contraction = args.max_contraction

    reader = EMGSerialReader(
        port=args.port,
        baud_rate=args.baud,
        rest_baseline=args.rest_baseline,
        ref_contraction=args.ref_contraction,
        threshold_light=args.threshold_light,
        threshold_strong=args.threshold_strong,
        release_ratio=args.release_ratio,
        smoothing_alpha=args.smoothing,
    )
    classifier = EMGMLClassifier(model_path=args.model)

    # ESP32(EMG) 포트를 먼저 열고 잠깐 대기한 뒤 AmazingHand 포트를 연다.
    # macOS에서 USB-시리얼 포트 2개를 거의 동시에 열면 termios.error가
    # 나는 경우가 있어서, 순서를 두고 안정화 시간을 준다.
    reader.start()
    print(f"[main] 시리얼 연결됨: {args.port} @ {args.baud}bps. Ctrl+C로 종료.")
    time.sleep(0.5)

    if args.calibrate:
        from emg_pipeline.calibration import auto_calibrate

        reader.rest_baseline, reader.ref_contraction = auto_calibrate(reader, seconds=args.calibrate_seconds)
    elif args.use_saved:
        from emg_pipeline.calibration import load_calibration

        saved = load_calibration()
        if saved is None:
            raise SystemExit("[main] calibration.json 이 없습니다. --calibrate 로 먼저 보정하세요.")
        reader.rest_baseline, reader.ref_contraction = saved
    else:
        print(f"[main] 경고: 보정 없이 기본값(rest={reader.rest_baseline}, ref={reader.ref_contraction})으로 실행합니다. "
              "--calibrate 권장.")

    hand_driver = None
    if args.hand_port:
        from emg_pipeline.amazinghand_driver import AmazingHandDriver

        hand_driver = AmazingHandDriver(port=args.hand_port)
        print(f"[main] AmazingHand 연결됨: {args.hand_port}")
    robot = RobotArmController(hand_driver=hand_driver)
    latch = GripLatch()

    last_print = 0.0
    try:
        while True:
            command = latch.update(reader.current_state)
            robot.apply(command)  # 명령이 실제로 바뀔 때만 [RobotArm] -> ... 한 줄 찍힘

            now = time.monotonic()
            if now - last_print >= args.print_interval:
                last_print = now
                window = reader.try_get_window()
                ml_state, ml_proba = ("(대기중)", 0.0)
                if window is not None:
                    ml_state, ml_proba = classifier.classify(window)

                # 같은 줄을 덮어써서 화면이 안 정신없게 (제어는 --interval대로 그대로 빠르게 돔)
                line = (
                    f"raw={reader.current_value:5d} norm={reader.current_normalized:.2f} "
                    f"state={reader.current_state:6s} [dbg ml={ml_state:9s}(p={ml_proba:.2f})] "
                    f"command={command.value:9s}"
                )
                print("\r" + line.ljust(100), end="", flush=True)

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[main] 종료합니다.")
    finally:
        reader.stop()
        robot.shutdown()


if __name__ == "__main__":
    main()
