"""
EMG 센서가 살아있는지 10초 만에 확인하는 진단 도구.

본격 수집 전에 이걸 먼저 돌려서, 신호가 실제로 근육에 반응하는지 확인한다.
(2026-09-12: 표준편차 5짜리 죽은 신호를 수집하고 나서야 알아챈 일이 있어서 추가)

판정 기준 (2026-09-22: "센서가 살아 있는가"만 본다. 잡음 대비 상승폭·스파이크는 판정에 안 씀):
  - GRIP 표준편차가 너무 작으면(<20) 살아있는 EMG가 아니라 고정 전압
  - GRIP 이 전부 0이면 센서 출력 없음
  - 회차 단위 품질(접촉 불량·붕괴)은 collect.py 의 자동 거부가 맡는다

사용법:
  python3 sensor_check.py --port /dev/cu.usbserial-110
"""

import argparse
import statistics
import time

import serial


def parse_args():
    p = argparse.ArgumentParser(description="EMG 센서 상태 점검")
    p.add_argument("--port", default="/dev/cu.usbserial-110")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--seconds", type=float, default=5.0, help="각 단계 측정 시간(초)")
    return p.parse_args()


def measure(ser, seconds, label):
    ser.reset_input_buffer()
    values = []
    t0 = time.monotonic()
    last_print = 0.0
    while time.monotonic() - t0 < seconds:
        line = ser.readline().decode(errors="ignore").strip()
        if "," not in line:
            continue
        try:
            v = int(line.split(",", 1)[1])
        except ValueError:
            continue
        if not (0 <= v <= 4095):   # 깨진 줄에서 나온 범위 밖 값 무시
            continue
        values.append(v)
        now = time.monotonic()
        if now - last_print >= 0.25:      # 0.25초마다 현재값을 막대로 표시
            last_print = now
            bar = "█" * int(v / 4095 * 40)
            print(f"\r   {label}: {v:5d} |{bar:<40s}|", end="", flush=True)
    print()
    return values


def stats(values):
    if not values:
        return None
    return {
        "n": len(values),
        "mean": statistics.mean(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def main():
    args = parse_args()
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)

    print("\n[1/2] 힘을 완전히 빼고 가만히 계세요")
    time.sleep(1)
    rest = stats(measure(ser, args.seconds, "REST"))

    print("\n[2/2] 이번엔 최대한 세게 주먹을 쥐세요")
    time.sleep(1)
    grip = stats(measure(ser, args.seconds, "GRIP"))

    ser.close()

    if rest is None or grip is None:
        print("\n❌ 데이터를 못 받았습니다. 포트/연결 확인 필요")
        return

    print("\n" + "=" * 52)
    print(f"  {'':6s} {'평균':>8s} {'표준편차':>9s} {'최소':>7s} {'최대':>7s}")
    print(f"  {'REST':6s} {rest['mean']:8.0f} {rest['std']:9.0f} {rest['min']:7d} {rest['max']:7d}")
    print(f"  {'GRIP':6s} {grip['mean']:8.0f} {grip['std']:9.0f} {grip['min']:7d} {grip['max']:7d}")
    print("=" * 52)

    # (2026-09-22) 판정은 "센서가 살아 있고 수치가 나오는가"만 본다.
    # 잡음 대비 상승폭(separation) 기준과 순간 스파이크는 판정에 쓰지 않는다.
    # 회차 단위 품질은 collect.py 의 자동 거부가 맡는다.
    rise = grip["mean"] - rest["mean"]
    headroom = 4095 - rest["mean"]          # 위로 쓸 수 있는 여유 폭

    problems, warnings = [], []
    if grip["std"] < 20:
        problems.append(f"GRIP 표준편차가 {grip['std']:.0f}뿐 → 살아있는 EMG가 아니라 고정 전압으로 보임")
    if grip["max"] == 0:
        problems.append("GRIP 이 전부 0 → 센서 출력이 없음 (전원/ENV 핀 배선 확인)")
    if rest["mean"] > 1500:
        warnings.append(
            f"REST 기준선이 {rest['mean']:.0f}로 높습니다 (위쪽 여유 {headroom:.0f}). "
            f"동작은 하지만 표현 범위가 좁아 분해능이 떨어집니다 — 접지/간섭 확인 권장"
        )

    if problems:
        print("\n❌ 센서에 문제가 있습니다. 수집하지 마세요.\n")
        for p in problems:
            print(f"   · {p}")
        print("\n   확인: REF 전극 부착 상태 → 전극 3개 스냅 연결 → 젤 마름 여부 → ENV 핀 배선")
    else:
        print(f"\n✅ 센서 작동 정상. REST 평균 {rest['mean']:.0f} → GRIP 평균 {grip['mean']:.0f} "
              f"(상승폭 {rise:.0f}).")
        for w in warnings:
            print(f"   ⚠️  {w}")
    print()


if __name__ == "__main__":
    main()
