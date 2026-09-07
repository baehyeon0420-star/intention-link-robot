"""
Unity 없이 파이썬만으로 돌아가는 intention-link 실시간 EMG -> 로봇팔 제어 루프.

구성 (Unity 버전과의 대응):
  EMGSerialReader.cs      -> emg_pipeline/serial_reader.py
  EMGMLClassifier.cs      -> emg_pipeline/ml_classifier.py
  RobotCommandMapper.cs   -> emg_pipeline/command_mapper.py (map_to_robot_command)
  EMGStateManager.cs      -> emg_pipeline/command_mapper.py (ContractionPatternDetector)
  RobotArmController.cs   -> emg_pipeline/robot_controller.py

실제 로봇 명령은 (Unity 원본과 동일하게) threshold 기반 상태(EMGSerialReader의
LIGHT/STRONG/GRIP)로 내려지고, ML 분류기(final_model.npz)는 비교용으로 같이
출력된다.
"""

import argparse
import time

from emg_pipeline.command_mapper import map_to_robot_command
from emg_pipeline.ml_classifier import EMGMLClassifier
from emg_pipeline.robot_controller import RobotArmController
from emg_pipeline.serial_reader import EMGSerialReader


def parse_args():
    p = argparse.ArgumentParser(description="intention-link 실시간 EMG 제어 (Unity 미사용)")
    p.add_argument("--port", required=True, help="ESP32(EMG) 시리얼 포트, 예: /dev/cu.usbserial-1110")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--rest-baseline", type=int, default=18)
    p.add_argument("--max-contraction", type=int, default=1321)
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

    reader = EMGSerialReader(
        port=args.port,
        baud_rate=args.baud,
        rest_baseline=args.rest_baseline,
        max_contraction=args.max_contraction,
    )
    classifier = EMGMLClassifier(model_path=args.model)

    hand_driver = None
    if args.hand_port:
        from emg_pipeline.amazinghand_driver import AmazingHandDriver

        hand_driver = AmazingHandDriver(port=args.hand_port)
        print(f"[main] AmazingHand 연결됨: {args.hand_port}")
    robot = RobotArmController(hand_driver=hand_driver)

    reader.start()
    print(f"[main] 시리얼 연결됨: {args.port} @ {args.baud}bps. Ctrl+C로 종료.")

    last_print = 0.0
    try:
        while True:
            command = map_to_robot_command(reader.current_state)
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
                    f"threshold={reader.current_state:7s} ml={ml_state:9s}(p={ml_proba:.2f}) "
                    f"command={command.value:9s}"
                )
                print("\r" + line.ljust(80), end="", flush=True)

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[main] 종료합니다.")
    finally:
        reader.stop()
        robot.shutdown()


if __name__ == "__main__":
    main()
