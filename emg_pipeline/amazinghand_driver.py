"""
EMG 파이프라인의 RobotCommand(RELEASE/HOLD/GRIP_CLOSE)를 실제 AmazingHand
(Pollen Robotics, Feetech SCS0009 x8, rustypot)로 전달하는 드라이버.

각도값은 amazinghand-setup 저장소의 AmazingHand_Demo_All.py에서 실측/캘리브레이션한
값을 그대로 옮겨온 것 (https://github.com/baehyeon0420-star/amazinghand-setup).
서보를 다시 조립하거나 캘리브레이션이 바뀌면 이 값도 같이 업데이트해야 함.
"""

import time

import numpy as np
from rustypot import Scs0009PyController

# ID별 (열림 각도, 닫힘 각도), 단위 도. amazinghand-setup/AmazingHand_Demo_All.py 참고.
OPEN_DEG = {1: -30, 2: 30, 3: 144, 4: -30, 5: 141, 6: -30, 7: 144, 8: -30}
CLOSE_DEG = {1: 90, 2: -90, 3: 54, 4: 90, 5: 24, 6: 118, 7: 54, 8: 90}
# HOLD: Release/GripClose 중간값 (절반만 오므림). 필요하면 튜닝.
HOLD_DEG = {sid: (OPEN_DEG[sid] + CLOSE_DEG[sid]) / 2 for sid in OPEN_DEG}


class AmazingHandDriver:
    """RobotArmController가 기대하는 release()/hold()/grip_close()/shutdown() 인터페이스."""

    def __init__(self, port, baudrate=1000000, speed=4):
        self.c = Scs0009PyController(serial_port=port, baudrate=baudrate, timeout=0.5)
        self.speed = speed
        self._torque_on = False

    def _robust(self, fn, tries=8, delay=0.05):
        last_err = None
        for _ in range(tries):
            try:
                return fn()
            except Exception as e:
                last_err = e
                time.sleep(delay)
        print(f"[AmazingHandDriver] 경고: 재시도 {tries}회 후 실패: {last_err!r}")
        return None

    def _ensure_torque(self):
        """토크를 켤 때 서보가 엉뚱한 위치로 튀지 않도록, 현재 위치를 먼저
        목표각도로 써준 뒤 토크를 켠다."""
        if self._torque_on:
            return
        for sid in OPEN_DEG:
            pos = self._robust(lambda sid=sid: self.c.read_present_position(sid))
            if pos is not None:
                self._robust(lambda sid=sid, pos=pos: self.c.write_goal_position(sid, pos[0]))
            self._robust(lambda sid=sid: self.c.write_torque_enable(sid, 1))
        self._torque_on = True

    def _move_all(self, deg_table):
        self._ensure_torque()
        for sid in deg_table:
            self._robust(lambda sid=sid: self.c.write_goal_speed(sid, self.speed))
        for sid, deg in deg_table.items():
            self._robust(lambda sid=sid, deg=deg: self.c.write_goal_position(sid, np.deg2rad(deg)))

    def release(self):
        self._move_all(OPEN_DEG)

    def hold(self):
        self._move_all(HOLD_DEG)

    def grip_close(self):
        self._move_all(CLOSE_DEG)

    def shutdown(self):
        """모든 서보 토크 해제. 프로그램 종료 시 반드시 호출할 것
        (안 그러면 손가락이 마지막 목표각도를 계속 힘줘서 유지함)."""
        for sid in OPEN_DEG:
            self._robust(lambda sid=sid: self.c.write_torque_enable(sid, 0))
        self._torque_on = False
