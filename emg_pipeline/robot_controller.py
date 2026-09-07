"""
로봇팔(그리퍼)에 명령을 내리는 출력단.

Unity 버전(RobotArmController.cs)은 3D 모델 회전으로 시각화했지만, 여기서는
실제 하드웨어(AmazingHand)가 정해졌으므로 hand_driver로 연결한다.
hand_driver는 release()/hold()/grip_close()/shutdown() 메서드를 갖는 객체면
무엇이든 꽂을 수 있음 (AmazingHandDriver가 기본 구현).
"""

from emg_pipeline.command_mapper import RobotCommand


class RobotArmController:
    def __init__(self, hand_driver=None):
        """hand_driver: emg_pipeline.amazinghand_driver.AmazingHandDriver 같은,
        release()/hold()/grip_close() 메서드를 가진 객체.
        없으면(None) 콘솔 출력만 함 (하드웨어 없이 파이프라인 테스트용).
        """
        self.hand_driver = hand_driver
        self.current_command = None

    def apply(self, command):
        if command == self.current_command:
            return
        self.current_command = command
        print(f"[RobotArm] -> {command.value}")
        self._send_to_hardware(command)

    def _send_to_hardware(self, command):
        if self.hand_driver is None:
            return
        if command == RobotCommand.RELEASE:
            self.hand_driver.release()
        elif command == RobotCommand.HOLD:
            self.hand_driver.hold()
        elif command == RobotCommand.GRIP_CLOSE:
            self.hand_driver.grip_close()

    def shutdown(self):
        """프로그램 종료 시 호출. 서보 토크를 해제해서 손가락이 계속
        힘주고 있지 않도록 한다."""
        if self.hand_driver is not None:
            self.hand_driver.shutdown()
