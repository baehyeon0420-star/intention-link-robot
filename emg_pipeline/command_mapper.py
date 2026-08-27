"""
EMG 상태를 로봇 명령/제스처 이벤트로 변환.

RobotCommandMapper.cs + EMGStateManager.cs를 이식.
"""

import time
from enum import Enum


class RobotCommand(Enum):
    RELEASE = "Release"
    HOLD = "Hold"
    GRIP_CLOSE = "GripClose"


def map_to_robot_command(threshold_state):
    """RobotCommandMapper.cs와 동일한 매핑.

    REST   -> Release
    LIGHT  -> Hold
    STRONG -> GripClose
    GRIP   -> GripClose
    """
    if threshold_state == "LIGHT":
        return RobotCommand.HOLD
    if threshold_state in ("STRONG", "GRIP"):
        return RobotCommand.GRIP_CLOSE
    return RobotCommand.RELEASE


class ContractionPatternDetector:
    """EMGStateManager.cs 이식 — Short/Long/Double 수축 패턴 감지.

    로봇 그리퍼 제어에는 직접 쓰이지 않고, 추후 XR/제스처 확장(Confirm/Hold/
    Release/Select/Menu 등)에 재사용하기 위해 원본 그대로 옮겨둔 것.
    """

    def __init__(self, short_max_ms=350, long_min_ms=700, double_window_ms=500):
        self.short_max_ms = short_max_ms
        self.long_min_ms = long_min_ms
        self.double_window_ms = double_window_ms

        self._was_active = False
        self._active_start = 0.0
        self._last_short_end = -999.0
        self._waiting_for_double = False

    def update(self, threshold_state):
        """매 tick 호출. 발생한 이벤트 이름 리스트를 반환 (없으면 빈 리스트).

        가능한 이벤트: "activation_started", "activation_ended",
        "short", "long", "double"
        """
        events = []
        now = time.time()
        is_active = threshold_state in ("LIGHT", "STRONG", "GRIP")

        if is_active and not self._was_active:
            self._active_start = now
            events.append("activation_started")

        if not is_active and self._was_active:
            duration_ms = (now - self._active_start) * 1000.0
            events.append("activation_ended")

            if duration_ms >= self.long_min_ms:
                events.append("long")
                self._waiting_for_double = False
            elif duration_ms < self.short_max_ms:
                if self._waiting_for_double and (now - self._last_short_end) * 1000.0 < self.double_window_ms:
                    events.append("double")
                    self._waiting_for_double = False
                else:
                    self._waiting_for_double = True
                    self._last_short_end = now
            # short_max_ms <= duration < long_min_ms 구간은 노이즈로 간주

        if self._waiting_for_double and (now - self._last_short_end) * 1000.0 >= self.double_window_ms:
            events.append("short")
            self._waiting_for_double = False

        self._was_active = is_active
        return events
