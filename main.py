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
    p.add_argument("--port", required=True, help="ESP32 시리얼 포트, 예: /dev/cu.usbserial-1110")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--rest-baseline", type=int, default=18)
    p.add_argument("--max-contraction", type=int, default=1321)
    p.add_argument("--model", default="final_model.npz")
    p.add_argument("--interval", type=float, default=0.05, help="루프 주기(초), 기본 50ms")
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
    robot = RobotArmController()

    reader.start()
    print(f"[main] 시리얼 연결됨: {args.port} @ {args.baud}bps. Ctrl+C로 종료.")

    try:
        while True:
            command = map_to_robot_command(reader.current_state)
            robot.apply(command)

            window = reader.try_get_window()
            ml_state, ml_proba = ("(대기중)", 0.0)
            if window is not None:
                ml_state, ml_proba = classifier.classify(window)

            print(
                f"raw={reader.current_value:5d} norm={reader.current_normalized:.2f} "
                f"threshold={reader.current_state:7s} ml={ml_state:9s}(p={ml_proba:.2f}) "
                f"command={command.value}"
            )

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[main] 종료합니다.")
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
