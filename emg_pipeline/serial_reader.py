"""
ESP32로부터 EMG 원시 신호를 시리얼로 읽어들이는 백그라운드 리더.

intention-link(Unity)의 EMGSerialReader.cs를 그대로 이식함:
  - "t_ms,raw" 또는 "raw" 단독 형식 모두 허용 (collect.py와 동일 포맷)
  - restBaseline~maxContraction 구간으로 0~1 정규화
  - thresholdLight/Strong/Grip 기준으로 REST/LIGHT/STRONG/GRIP 상태 분류
  - ML 분류기(ml_classifier.py)가 쓰는 최근 20샘플(200ms) 윈도우 유지
"""

import threading
import time
from collections import deque

import serial


class EMGSerialReader:
    # features.py의 WIN=20(=200ms, 100Hz 가정)과 동일
    WINDOW_SIZE = 20

    def __init__(
        self,
        port,
        baud_rate=115200,
        rest_baseline=18,
        max_contraction=1321,
        threshold_light=0.1,
        threshold_strong=0.4,
        threshold_grip=0.7,
    ):
        self.port_name = port
        self.baud_rate = baud_rate
        self.rest_baseline = rest_baseline
        self.max_contraction = max_contraction
        self.threshold_light = threshold_light
        self.threshold_strong = threshold_strong
        self.threshold_grip = threshold_grip

        self.current_value = 0
        self.current_normalized = 0.0
        self.current_state = "REST"

        self._window = deque(maxlen=self.WINDOW_SIZE)
        self._lock = threading.Lock()
        self._running = False
        self._port = None
        self._thread = None

    def start(self):
        self._port = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
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

        normalized = self._normalize(value)
        state = self._classify(normalized)

        with self._lock:
            self.current_value = value
            self.current_normalized = normalized
            self.current_state = state
            self._window.append(value)

    def _normalize(self, value):
        span = self.max_contraction - self.rest_baseline
        if span <= 0:
            return 0.0
        return min(1.0, max(0.0, (value - self.rest_baseline) / span))

    def _classify(self, normalized):
        if normalized >= self.threshold_grip:
            return "GRIP"
        if normalized >= self.threshold_strong:
            return "STRONG"
        if normalized >= self.threshold_light:
            return "LIGHT"
        return "REST"

    def try_get_window(self):
        """최근 WINDOW_SIZE개의 raw 샘플을 오래된 순서로 반환. 아직 안 찼으면 None."""
        with self._lock:
            if len(self._window) < self.WINDOW_SIZE:
                return None
            return list(self._window)
