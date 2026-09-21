"""
EMG rest_baseline / max_contraction 자동 캘리브레이션.

전극 접촉 상태(땀, 밀착도, 붙인 위치)가 세션마다 조금씩 달라져서 raw 신호의
기준선이 매번 바뀐다. 그때마다 값을 손으로 추측해 넣는 대신, 시작할 때
"힘 빼기"/"최대한 세게 쥐기" 몇 초씩 측정해서 자동으로 잡는다.

(2026-09-21) max_contraction은 순간 최고치가 아니라 쥐고 있는 동안의 중앙값으로 잡는다.
ENV 출력은 쥐고 있는 동안에도 순간적으로 크게 튀어서(v5: 중앙값 1,040 / 최고 2,752),
최고치 기준이면 100% 지점이 실제 힘보다 한참 위에 잡힌다. 그 결과 쥐기 판정이 최대
힘 평균의 ~70% 이상에서야 나와서, 로봇손을 움직이려면 손을 정말 꽉 쥐어야 했다.
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
    max_contraction = int(statistics.median(max_samples))
    print(f"[calibrate] 중앙값={max_contraction} (순간 최고={max(max_samples)}) -> max_contraction={max_contraction}")

    # sensor_check.py와 같은 판정: 쥐기 상승폭이 rest 잡음의 3배도 안 되면 근육 반응이 묻힌 것.
    separation = (max_contraction - rest_mean) / max(rest_std, 1)
    if max_contraction <= rest_baseline or separation < 3:
        print(
            f"[calibrate] 경고: 쥐기 상승폭이 rest 잡음의 {separation:.1f}배뿐입니다. "
            "전극 접촉(REF 위치, 젤 마름)을 확인하고 다시 시도하세요."
        )

    print(f"[calibrate] 완료: rest_baseline={rest_baseline}, max_contraction={max_contraction}\n")
    return rest_baseline, max_contraction
