"""
EMG rest_baseline / max_contraction 자동 캘리브레이션.

전극 접촉 상태(땀, 밀착도, 붙인 위치)가 세션마다 조금씩 달라져서 raw 신호의
기준선이 매번 바뀐다. 그때마다 값을 손으로 추측해 넣는 대신, 시작할 때
"힘 빼기"/"최대한 세게 쥐기" 몇 초씩 측정해서 자동으로 잡는다.
"""

import statistics
import time


def _collect(reader, seconds):
    samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        samples.append(reader.current_value)
        time.sleep(0.02)
    return samples


def auto_calibrate(reader, seconds=3.0):
    """reader가 이미 start()된 상태에서 호출. (rest_baseline, max_contraction) 반환."""
    print(f"\n[calibrate] 힘을 완전히 빼고 가만히 계세요... ({seconds:.0f}초)")
    time.sleep(1.0)  # 안내 읽고 준비할 시간
    rest_samples = _collect(reader, seconds)
    rest_mean = statistics.mean(rest_samples)
    rest_std = statistics.pstdev(rest_samples) if len(rest_samples) > 1 else 0.0
    rest_baseline = int(rest_mean + rest_std)
    print(f"[calibrate] REST 평균={rest_mean:.0f}, 표준편차={rest_std:.0f} -> rest_baseline={rest_baseline}")

    print(f"\n[calibrate] 이번엔 최대한 세게 주먹을 쥐세요... ({seconds:.0f}초)")
    time.sleep(1.0)
    max_samples = _collect(reader, seconds)
    peak = max(max_samples)
    max_contraction = int(peak * 0.9)  # 매번 절대 최고치까지 안 쥐어도 GRIP 나오게 약간 여유
    print(f"[calibrate] 최고값={peak} -> max_contraction={max_contraction}")

    if max_contraction <= rest_baseline:
        print(
            "[calibrate] 경고: max_contraction이 rest_baseline보다 작거나 같습니다. "
            "전극 접촉을 확인하고 다시 시도하세요."
        )

    print(f"[calibrate] 완료: rest_baseline={rest_baseline}, max_contraction={max_contraction}\n")
    return rest_baseline, max_contraction
