"""
로봇팔(그리퍼)에 명령을 내리는 출력단.

Unity 버전(RobotArmController.cs)은 3D 모델 회전으로 시각화했지만, 여기서는
실물 로봇팔이 아직 정해지지 않았으므로 우선 콘솔 출력 + 상태 보관만 하고,
실제 하드웨어 프로토콜(시리얼/PWM/BLE 등)이 정해지면 _send_to_hardware()만
채우면 되도록 분리해둠.
"""


class RobotArmController:
    def __init__(self, hardware_port=None):
        """hardware_port: 로봇팔 제어보드로 나가는 시리얼 포트(pyserial.Serial).
        아직 없으면 None으로 두면 콘솔 출력만 함.
        """
        self.hardware_port = hardware_port
        self.current_command = None

    def apply(self, command):
        if command == self.current_command:
            return
        self.current_command = command
        print(f"[RobotArm] -> {command.value}")
        self._send_to_hardware(command)

    def _send_to_hardware(self, command):
        if self.hardware_port is None:
            return
        # TODO: 실제 로봇팔/그리퍼 제어보드의 통신 프로토콜이 정해지면 여기서 전송.
        # 예: self.hardware_port.write(f"{command.value}\n".encode())
        pass
