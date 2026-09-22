"""
EMG 센서가 살아있는지 10초 만에 확인하는 진단 도구.

본격 수집 전에 이걸 먼저 돌려서, 신호가 실제로 근육에 반응하는지 확인한다.
(2026-09-12: 표준편차 5짜리 죽은 신호를 수집하고 나서야 알아챈 일이 있어서 추가)

판정 기준 (2026-09-22: 0값과 4095 스파이크를 뺀 파형의 중앙값으로 "센서가 살아 있는가"만 본다):
  - GRIP 유효 샘플이 없으면(전부 0/4095) 파형 없음
  - GRIP 파형 표준편차가 너무 작으면(<20) 고정 전압
  - GRIP 중앙값이 REST 중앙값보다 크지 않으면 근육에 반응하지 않음
  - 잡음 대비 배수 기준은 쓰지 않음. 회차 단위 품질은 collect.py 의 자동 거부가 맡는다

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
    window = []                       # 막대 표시용 0.25초 구간
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
        window.append(v)
        now = time.monotonic()
        if now - last_print >= 0.25:      # 0.25초마다 구간 중앙값을 막대로 표시 (마지막 샘플 하나가 아님)
            last_print = now
            valid = [x for x in window if 0 < x < 4095]
            shown = int(statistics.median(valid)) if valid else 0
            window = []
            bar = "█" * int(shown / 4095 * 40)
            print(f"\r   {label}: {shown:5d} |{bar:<40s}|", end="", flush=True)
    print()
    return values


def stats(values, drop_zero=True):
    """4095(포화 스파이크)와, drop_zero 면 0(ADC 바닥 = 접촉 끊김)도 뺀 파형으로 통계를 낸다.
    (2026-09-22) 접촉이 들락날락하면 0과 4095가 섞여 평균·표준편차가 무의미해진다.
    GRIP 은 0을 빼고(끊김), REST 는 0을 남긴다(0이 실제 기준선인 경우가 대부분).
    판정은 valid 구간의 중앙값으로 하고, 전체 대비 valid 비율은 참고로만 보여준다."""
    if not values:
        return None
    lo = 0 if drop_zero else -1
    valid = [v for v in values if lo < v < 4095]
    if not valid:
        return {"n": len(values), "valid_pct": 0.0, "median": 0, "std": 0.0, "min": 0, "max": 0,
                "raw_mean": statistics.mean(values)}
    return {
        "n": len(values),
        "valid_pct": len(valid) / len(values) * 100,
        "median": statistics.median(valid),
        "std": statistics.pstdev(valid) if len(valid) > 1 else 0.0,
        "min": min(valid),
        "max": max(valid),
        "raw_mean": statistics.mean(values),
    }


def main():
    args = parse_args()
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)

    print("\n[1/2] 힘을 완전히 빼고 가만히 계세요")
    time.sleep(1)
    rest = stats(measure(ser, args.seconds, "REST"), drop_zero=False)

    print("\n[2/2] 이번엔 최대한 세게 주먹을 쥐세요")
    time.sleep(1)
    grip = stats(measure(ser, args.seconds, "GRIP"))

    ser.close()

    if rest is None or grip is None:
        print("\n❌ 데이터를 못 받았습니다. 포트/연결 확인 필요")
        return

    print("\n" + "=" * 62)
    print(f"  {'':6s} {'중앙값':>7s} {'표준편차':>8s} {'최소':>6s} {'최대':>6s} {'유효%':>6s}   (REST 4095 제외, GRIP 0·4095 제외)")
    print(f"  {'REST':6s} {rest['median']:7.0f} {rest['std']:8.0f} {rest['min']:6d} {rest['max']:6d} {rest['valid_pct']:6.0f}")
    print(f"  {'GRIP':6s} {grip['median']:7.0f} {grip['std']:8.0f} {grip['min']:6d} {grip['max']:6d} {grip['valid_pct']:6.0f}")
    print("=" * 62)

    # (2026-09-22) 판정은 0과 4095 스파이크를 뺀 파형의 중앙값으로 한다.
    # "센서가 살아 있고 쥐면 올라가는가"만 본다. 잡음 대비 배수 기준은 쓰지 않는다.
    # 회차 단위 품질은 collect.py 의 자동 거부가 맡는다.
    rise = grip["median"] - rest["median"]
    headroom = 4095 - rest["median"]

    problems, warnings = [], []
    if grip["valid_pct"] == 0:
        problems.append("GRIP 이 전부 0 또는 4095 → 살아있는 파형이 없음 (전원/ENV 핀/전극 확인)")
    elif grip["std"] < 20:
        problems.append(f"GRIP 파형 표준편차가 {grip['std']:.0f}뿐 → 살아있는 EMG가 아니라 고정 전압으로 보임")
    elif rise <= 0:
        problems.append(f"쥐어도 중앙값이 안 오름 (REST {rest['median']:.0f} → GRIP {grip['median']:.0f}) → 근육에 반응하지 않음")
    if rest["median"] > 1500:
        warnings.append(
            f"REST 기준선이 {rest['median']:.0f}로 높습니다 (위쪽 여유 {headroom:.0f}). "
            f"동작은 하지만 표현 범위가 좁아 분해능이 떨어집니다 — 접지/간섭 확인 권장"
        )
    if grip["valid_pct"] < 50:
        warnings.append(f"GRIP 샘플의 {100-grip['valid_pct']:.0f}%가 0 또는 4095 → 판정은 나머지 {grip['valid_pct']:.0f}%로만 했음")

    if problems:
        print("\n❌ 센서에 문제가 있습니다. 수집하지 마세요.\n")
        for p in problems:
            print(f"   · {p}")
        print("\n   확인: REF 전극 부착 상태 → 전극 3개 스냅 연결 → 젤 마름 여부 → ENV 핀 배선")
    else:
        print(f"\n✅ 센서 작동 정상. REST 중앙값 {rest['median']:.0f} → GRIP 중앙값 {grip['median']:.0f} "
              f"(상승폭 {rise:.0f}).")
    for w in warnings:
        print(f"   ⚠️  {w}")
    print()


if __name__ == "__main__":
    main()
