"""
EMG 원시 신호 수집.

개선된 프로토콜(2026-09-12): 한 번에 길게 받으면 근피로로 뒤로 갈수록 신호가
빠져서(s3 사례: 2587 -> 672 -> 356 -> 371) 라벨이 오염된다. 그래서
"짧게 여러 번 + 사이에 휴식"으로 나눠 받고, 매 회차 평균을 즉시 찍어서
힘이 빠지고 있으면 수집 중에 바로 알 수 있게 한다.

사용 예:
  # 휴식 데이터 (한 번에 길게 받아도 됨 - 힘을 안 주니 피로가 없음)
  python3 collect.py --port /dev/cu.usbserial-110 --subject s5 --gesture rest --seconds 10 --reps 1

  # 쥐기 데이터 (3초씩 6회, 사이에 5초 휴식)
  python3 collect.py --port /dev/cu.usbserial-110 --subject s5 --gesture grip --seconds 3 --reps 6 --rest-between 5

저장: "{subject}_{gesture}.csv" (있으면 이어쓰기)
"""

import argparse
import csv
import os
import statistics
import threading
import time

import serial


class SerialPump:
    """포트를 연 순간부터 쉬지 않고 읽어서 버퍼가 밀리지 않게 하는 백그라운드 리더.

    (2026-09-12) 기존에는 sleep 후 reset_input_buffer()로 비우려 했는데, macOS
    USB-시리얼 드라이버 버퍼까지는 비워지지 않아서 측정 시작 직후 7초치 과거
    데이터가 먼저 읽혔다. 그래서 serial_reader.py처럼 항상 읽어 두는 구조로 바꿈.
    """

    def __init__(self, ser):
        self.ser = ser
        self._buf = []
        self._capturing = False
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if "," not in line:
                continue
            t, raw = line.split(",", 1)
            try:
                v = int(raw)
            except ValueError:
                continue
            # ESP32 ADC는 12비트(0~4095). 깨진 줄에서 나온 범위 밖 값은 버린다.
            if not (0 <= v <= 4095):
                continue
            with self._lock:
                if self._capturing:
                    self._buf.append(([t, raw], v))

    def capture(self, seconds):
        """지금부터 seconds 동안 들어오는 '실시간' 데이터만 모은다."""
        with self._lock:
            self._buf = []
            self._capturing = True
        time.sleep(seconds)
        with self._lock:
            self._capturing = False
            out = list(self._buf)
        return [r for r, _ in out], [v for _, v in out]

    def stop(self):
        self._running = False
        self._thread.join(timeout=0.5)


def parse_args():
    p = argparse.ArgumentParser(description="EMG raw 신호 수집 (반복 + 피로 감시)")
    p.add_argument("--port", default="/dev/cu.usbserial-110")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--subject", required=True, help="예: s1, s5 ...")
    p.add_argument("--gesture", required=True, help="예: rest, grip, light50 ...")
    p.add_argument("--seconds", type=float, default=3.0, help="1회당 수집 시간(초)")
    p.add_argument("--reps", type=int, default=1, help="반복 횟수")
    p.add_argument("--rest-between", type=float, default=5.0, help="회차 사이 휴식(초)")
    p.add_argument("--countdown", type=int, default=3, help="각 회차 시작 전 준비시간(초)")
    p.add_argument(
        "--lead-in",
        type=float,
        default=1.0,
        help="'쥐세요' 신호 후 실제 측정을 시작하기까지 기다리는 시간(초). "
        "힘을 주는 상승 구간이 데이터에 섞이지 않게 하기 위함. rest 수집 땐 0으로.",
    )
    return p.parse_args()


def main():
    args = parse_args()
    fname = f"{args.subject}_{args.gesture}.csv"

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # ESP32 리셋 대기
    pump = SerialPump(ser)  # 이 시점부터 계속 읽어서 버퍼가 밀리지 않게 함

    all_rows = []
    rep_means = []

    print(f"\n=== {args.subject} / {args.gesture} : {args.seconds:.0f}초 x {args.reps}회 ===")
    for rep in range(1, args.reps + 1):
        print(f"\n[{rep}/{args.reps}] 준비...")
        for i in range(args.countdown, 0, -1):
            print(f"   {i}")
            time.sleep(1)
        if args.lead_in > 0:
            # 먼저 쥐게 하고, 힘이 다 올라온 뒤에 측정을 시작한다.
            print(f"   ▶ 지금 쥐세요!")
            time.sleep(args.lead_in)
            print(f"   ● 측정 중... ({args.seconds:.0f}초) — 그대로 유지")
        else:
            print(f"   ▶ 시작! ({args.seconds:.0f}초)")

        rows, values = pump.capture(args.seconds)
        all_rows.extend([r + [args.subject, args.gesture] for r in rows])

        if values:
            mean = statistics.mean(values)
            rep_means.append(mean)
            # 1회차 대비 얼마나 떨어졌는지 = 근피로 감시
            drop = ""
            if len(rep_means) > 1:
                ratio = mean / rep_means[0] if rep_means[0] else 0
                drop = f"  (1회차 대비 {ratio*100:.0f}%)"
                if ratio < 0.7:
                    drop += "  ⚠️ 힘이 빠지고 있음 — 더 쉬었다 하세요"
            print(f"   ✔ {len(values)}샘플, 평균={mean:.0f}{drop}")
        else:
            print("   ⚠ 수신된 데이터가 없습니다. 연결 확인 필요")

        if rep < args.reps and args.rest_between > 0:
            print(f"   ... {args.rest_between:.0f}초 휴식 (힘 완전히 빼세요)")
            time.sleep(args.rest_between)

    pump.stop()
    ser.close()

    file_exists = os.path.exists(fname)
    with open(fname, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["t_ms", "raw", "subject", "gesture"])
        w.writerows(all_rows)

    print(f"\n=== 완료: {len(all_rows)}샘플 -> {fname} ===")
    if len(rep_means) > 2:
        print(f"회차별 평균: {[f'{m:.0f}' for m in rep_means]}")

        # 회차 간 힘이 들쑥날쑥한 것 자체는 문제가 아니다(실사용에서도 매번 다르다).
        # 진짜 문제는 (1) 피로로 회차가 갈수록 단조롭게 무너지는 것,
        #            (2) 특정 회차가 다른 회차 대비 붕괴해서 라벨이 거짓이 되는 것.
        n = len(rep_means)
        xs = list(range(n))
        mx, my = sum(xs) / n, sum(rep_means) / n
        cov = sum((x - mx) * (y - my) for x, y in zip(xs, rep_means))
        vx = sum((x - mx) ** 2 for x in xs) ** 0.5
        vy = sum((y - my) ** 2 for y in rep_means) ** 0.5
        corr = cov / (vx * vy) if vx and vy else 0.0

        med = statistics.median(rep_means)
        worst = min(rep_means)

        # 방향(상관)만 보면 20% 정도 완만히 내려가는 정상 케이스도 걸린다.
        # s3의 붕괴는 4092 -> 42 (99% 감소)였으므로, 크기(마지막/처음)도 같이 본다.
        decline = rep_means[-1] / rep_means[0] if rep_means[0] else 1.0
        print(f"피로 추세(회차-평균 상관): {corr:+.2f}, 마지막/처음 = {decline:.2f}", end="")
        if corr <= -0.7 and decline < 0.6:
            print("  ⚠️ 회차가 갈수록 무너지고 있습니다 (s3 사례와 같은 패턴) — 다시 받으세요")
        elif corr <= -0.7:
            print("  (완만한 하강이지만 크기는 허용 범위)")
        else:
            print("  (추세 없음 = 정상)")

        print(f"최약 회차 / 중앙값: {worst / med:.2f}", end="")
        if worst < med * 0.4:
            print("  ⚠️ 유독 약한 회차가 있습니다 — 그 회차는 grip이라 보기 어려움")
        else:
            print("  (모든 회차가 grip 수준 유지 = 정상)")
    print()


if __name__ == "__main__":
    main()
