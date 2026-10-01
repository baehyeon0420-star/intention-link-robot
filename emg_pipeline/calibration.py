"""
EMG rest_baseline / ref_contraction 자동 캘리브레이션.

전극 접촉 상태(땀, 밀착도, 붙인 위치)가 세션마다 조금씩 달라져서 raw 신호의
기준선이 매번 바뀐다. 그때마다 값을 손으로 추측해 넣는 대신, 시작할 때
"힘 빼기"/"편하게 쥐기" 3초씩 측정해서 자동으로 잡는다.

(2026-09-22) 기준을 "최대한 세게 쥐기"에서 "물건 집듯이 편하게 쥐기"로 바꿈.
  - 세게 쥔 값을 100%로 잡으면 실제 사용 세기(살짝 쥐기)가 기준의 30~40%밖에 안 돼서
    임계값을 낮게 잡아야 했고, 보정할 때 지치고, 사람마다 "최대"의 편차도 컸다.
  - 이제 편하게 쥔 세기 자체가 1.0. 로봇 명령은 그 65%(threshold_strong)에서 쥐고, LIGHT 구간에서는
    이전 명령을 유지하다가 REST 로 내려가야 놓는다(command_mapper.GripLatch).
  - rest/ref 모두 중앙값으로 잡는다 (순간 스파이크에 안 흔들리게).
결과는 calibration.json에 저장한다. 다음 실행에서 --use-saved를 주면 다시 읽는다
(전극을 다시 붙이면 값이 달라지므로 자동으로 읽지는 않는다).
"""

import json
import os
import statistics
import time
from datetime import datetime

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calibration.json")
MIN_SEPARATION_STD = 3   # (ref - rest) 가 rest 표준편차의 이 배수 미만이면 전극 접촉 불량 (sensor_check.py 와 같은 기준)


def _countdown(n, msg):
    print(f"\n{msg}")
    for i in range(n, 0, -1):
        print(f"   {i}", flush=True)
        time.sleep(1.0)


def _collect(reader, seconds, label="측정 중"):
    """seconds 동안 샘플을 모으면서 남은 시간과 현재값을 같은 줄에 보여준다."""
    samples = []
    t0 = time.monotonic()
    last = -1
    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= seconds:
            break
        samples.append(reader.current_value)
        remain = int(seconds - elapsed) + 1
        if remain != last:
            last = remain
            bar = "█" * int((seconds - remain + 1) / seconds * 20)
            print(f"\r   ● {label}... 남은 {remain}초 |{bar:<20s}| 현재값 {reader.current_value:4d}   ", end="", flush=True)
        time.sleep(0.02)
    print(f"\r   ✔ {label} 끝                                            ")
    return samples


def auto_calibrate(reader, seconds=3.0, retries=2, save_path=CALIBRATION_FILE, countdown=3, lead_in=1.0):
    """reader가 이미 start()된 상태에서 호출. (rest_baseline, ref_contraction) 반환.

    안내 문구에 "세게"를 쓰지 않는다 — 편하게 쥔 세기가 기준이다.
    (2026-10-01) 단계 표시·카운트다운·남은 시간 표시 추가. 전에는 안내 1초 뒤 바로 측정이 시작돼
    언제 쥐어야 하는지 알 수 없었고, 쥐기 시작 구간이 측정에 섞였다.
    """
    print("\n==================== 보정 시작 ====================")
    print(f" 1단계: 힘 빼고 가만히 {seconds:.0f}초  →  2단계: 편하게 쥐고 {seconds:.0f}초")
    print(" 편하게 = 컵을 드는 정도. 세게 쥐지 마세요. 그 세기가 기준(1.0)이 됩니다.")
    print("===================================================")
    for attempt in range(1, retries + 2):
        if attempt > 1:
            print(f"\n[calibrate] 다시 시도 {attempt - 1}/{retries}. 전극을 한 번 눌러 붙이고 준비하세요.")
        _countdown(countdown, f"[1/2] 힘을 완전히 빼고 가만히 계세요. {countdown}초 뒤 측정 시작")
        rest_samples = _collect(reader, seconds, "힘 뺀 상태 측정 중")
        rest_baseline = int(statistics.median(rest_samples))
        rest_std = statistics.pstdev(rest_samples) if len(rest_samples) > 1 else 0.0
        print(f"   → REST 중앙값={rest_baseline}, 표준편차={rest_std:.0f}")

        _countdown(countdown, f"[2/2] 신호가 오면 물건 집듯이 편하게 쥐세요. {countdown}초 뒤 신호")
        print("   ▶ 지금 편하게 쥐세요! (컵 드는 힘, 그대로 유지)", flush=True)
        time.sleep(lead_in)  # 힘이 올라오는 구간은 측정에 안 넣는다
        ref_samples = _collect(reader, seconds, "편한 쥐기 측정 중 (그대로 유지)")
        print("   손에 힘을 빼세요.")
        ref_contraction = int(statistics.median(ref_samples))
        print(f"   → 편한 쥐기 중앙값={ref_contraction} (순간 최고={max(ref_samples)})")

        separation = (ref_contraction - rest_baseline) / max(rest_std, 1)
        if ref_contraction > rest_baseline and separation >= MIN_SEPARATION_STD:
            print(f"   ✔ 쥐기가 rest 보다 {ref_contraction - rest_baseline} 높음 (잡음의 {separation:.1f}배). 보정 성공.")
            break
        print(
            f"   ✘ 쥐기 상승폭이 rest 잡음의 {separation:.1f}배뿐입니다 (최소 {MIN_SEPARATION_STD}배). "
            "전극 접촉(REF 위치, 스냅, 젤 마름)을 확인하세요."
        )
    else:
        print("[calibrate] 경고: 재시도 후에도 차이가 작습니다. 이 값으로 진행하지만 판정이 불안정할 수 있습니다.")

    print(f"\n[calibrate] 완료: rest_baseline={rest_baseline}, ref_contraction={ref_contraction}")
    print(f"           닫힘 임계 ≈ rest + 0.7×(ref−rest) = {int(rest_baseline + 0.7*(ref_contraction-rest_baseline))}, "
          f"해제 ≈ {int(rest_baseline + 0.3*(ref_contraction-rest_baseline))}\n")
    if save_path:
        save_calibration(rest_baseline, ref_contraction, save_path)
    return rest_baseline, ref_contraction


def save_calibration(rest_baseline, ref_contraction, path=CALIBRATION_FILE):
    data = {
        "rest_baseline": int(rest_baseline),
        "ref_contraction": int(ref_contraction),
        "saved_at": datetime.now().isoformat(timespec="seconds"),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"[calibrate] 저장: {path}")


def load_calibration(path=CALIBRATION_FILE):
    """저장된 보정값. 없으면 None. 전극 재부착 후엔 맞지 않을 수 있으니 저장 시각을 같이 보여준다."""
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    print(f"[calibrate] 저장된 보정값 사용: rest={data['rest_baseline']} ref={data['ref_contraction']} "
          f"(저장 {data.get('saved_at', '?')}) — 전극을 다시 붙였다면 --calibrate 권장")
    return data["rest_baseline"], data["ref_contraction"]
