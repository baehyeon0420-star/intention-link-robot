"""
ESP32로부터 EMG 원시 신호를 시리얼로 읽어들이는 백그라운드 리더.

intention-link(Unity)의 EMGSerialReader.cs를 그대로 이식함:
  - "t_ms,raw" 또는 "raw" 단독 형식 모두 허용 (collect.py와 동일 포맷)
  - restBaseline~maxContraction 구간으로 0~1 정규화
  - thresholdLight/Strong/Grip 기준으로 REST/LIGHT/STRONG/GRIP 상태 분류
    (올라갈 땐 threshold, 내려갈 땐 threshold x release_ratio 밑으로 떨어져야 함 — 히스테리시스)
  - ML 분류기(ml_classifier.py)가 쓰는 최근 20샘플(200ms) 윈도우 유지
"""

import threading
import time
from collections import deque

import serial


class EMGSerialReader:
    # features.py의 WIN=20(=200ms, 100Hz 가정)과 동일
    WINDOW_SIZE = 20
    LEVELS = ("REST", "LIGHT", "STRONG", "GRIP")

    def __init__(
        self,
        port,
        baud_rate=115200,
        rest_baseline=18,
        max_contraction=1321,
        threshold_light=0.2,
        threshold_strong=0.35,
        threshold_grip=0.7,
        release_ratio=0.43,
        smoothing_alpha=0.02,
    ):
        self.port_name = port
        self.baud_rate = baud_rate
        self.rest_baseline = rest_baseline
        self.max_contraction = max_contraction
        self.threshold_light = threshold_light
        self.threshold_strong = threshold_strong
        self.threshold_grip = threshold_grip
        # (2026-09-21) threshold_light 0.1 → 0.2: 로봇 연결 상태 실측(run.log)에서 힘을 뺀 채로도
        # 신호가 튀어 0.1을 넘으면서 Release↔Hold(절반 오므림)가 25초에 수십 번 오갔음.
        # (2026-09-21) 한 번 올라간 상태는 threshold x release_ratio 밑으로 떨어져야 내려간다.
        # 기본값이면 STRONG(쥐기)은 0.35에서 들어가고 0.15까지는 유지 → 쥔 뒤엔 약하게만
        # 힘을 줘도 로봇손이 안 풀림. 경계값 근처에서 상태가 깜빡이는 것도 막는다.
        self.release_ratio = release_ratio
        # raw 신호는 순간순간 노이즈가 커서, 지수이동평균(EMA)으로 부드럽게 만든 뒤
        # 그 값으로 threshold 상태를 판단한다 (0~1, 작을수록 더 부드러움/느림).
        # 샘플이 500Hz로 들어오므로 0.02 ≈ 시간상수 100ms. (예전 0.15는 ≈13ms라 거의
        # 평활이 안 돼서 쥐고 있는 동안에도 상태가 계속 깜빡였음 — v5 기록 기준 99회 → 1회)
        self.smoothing_alpha = smoothing_alpha

        self.current_value = 0
        self.current_normalized = 0.0
        self.current_state = "REST"
        self._smoothed_value = None

        self._window = deque(maxlen=self.WINDOW_SIZE)
        self._lock = threading.Lock()
        self._running = False
        self._port = None
        self._thread = None

    def start(self, retries=5, retry_delay=0.5):
        # macOS + 일부 USB-CDC 시리얼 칩(ESP32 내장 USB 등) 조합에서, pyserial
        # 생성자에 port/baudrate를 한 번에 넘기면 termios.error: (22, 'Invalid
        # argument')가 나는 경우가 있음. 속성을 따로 설정하고 나중에 여는 방식이
        # 이 문제를 우회하는 경우가 많아서 이렇게 열고, 그래도 실패하면 재시도한다.
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                port = serial.Serial()
                port.port = self.port_name
                port.baudrate = self.baud_rate
                port.timeout = 0.1
                port.open()
                self._port = port
                break
            except Exception as e:
                last_err = e
                if attempt < retries:
                    time.sleep(retry_delay)
        else:
            raise last_err

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._port is not None and self._port.is_open:
            self._port.close()

    def _read_loop(self):
        while self._running and self._port.is_open:
            try:
                line = self._port.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue
            self._handle_line(line)

    def _handle_line(self, line):
        # 아두이노가 "시간ms,raw" 형식으로 보내면 두 번째 값만 사용 (콤마 없으면 그대로 raw)
        parts = line.split(",")
        raw_part = parts[1] if len(parts) >= 2 else parts[0]
        try:
            value = int(raw_part)
        except ValueError:
            return
        # ESP32 ADC는 12비트(0~4095). 깨진 줄에서 나온 범위 밖 값은 버린다 (collect.py와 동일).
        if not (0 <= value <= 4095):
            return

        if self._smoothed_value is None:
            self._smoothed_value = float(value)
        else:
            a = self.smoothing_alpha
            self._smoothed_value = a * value + (1 - a) * self._smoothed_value

        normalized = self._normalize(self._smoothed_value)
        state = self._classify(normalized)

        with self._lock:
            self.current_value = value  # 화면 표시는 raw 그대로
            self.current_normalized = normalized  # 상태 판단은 부드럽게 만든 값 기준
            self.current_state = state
            self._window.append(value)  # ML 특징추출용 윈도우는 raw 유지

    def _normalize(self, value):
        span = self.max_contraction - self.rest_baseline
        if span <= 0:
            return 0.0
        return min(1.0, max(0.0, (value - self.rest_baseline) / span))

    def _thresholds(self):
        return {"LIGHT": self.threshold_light, "STRONG": self.threshold_strong, "GRIP": self.threshold_grip}

    def _classify(self, normalized):
        thresholds = self._thresholds()
        rising = "REST"
        for level in self.LEVELS[1:]:
            if normalized >= thresholds[level]:
                rising = level

        current = self.current_state
        if self.LEVELS.index(rising) >= self.LEVELS.index(current):
            return rising
        # 내려가는 중: 현재 상태부터 한 단계씩 내려가며, 아직 해제 기준 이상인 상태에 머문다.
        for idx in range(self.LEVELS.index(current), self.LEVELS.index(rising), -1):
            level = self.LEVELS[idx]
            if normalized >= thresholds[level] * self.release_ratio:
                return level
        return rising

    def try_get_window(self):
        """최근 WINDOW_SIZE개의 raw 샘플을 오래된 순서로 반환. 아직 안 찼으면 None."""
        with self._lock:
            if len(self._window) < self.WINDOW_SIZE:
                return None
            return list(self._window)
