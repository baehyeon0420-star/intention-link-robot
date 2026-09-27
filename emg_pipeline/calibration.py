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


def _collect(reader, seconds):
    samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        samples.append(reader.current_value)
        time.sleep(0.02)
    return samples


def auto_calibrate(reader, seconds=3.0, retries=2, save_path=CALIBRATION_FILE):
    """reader가 이미 start()된 상태에서 호출. (rest_baseline, ref_contraction) 반환.

    안내 문구에 "세게"를 쓰지 않는다 — 편하게 쥔 세기가 기준이다.
    """
    for attempt in range(1, retries + 2):
        print(f"\n[calibrate] 힘을 완전히 빼고 {seconds:.0f}초 유지하세요...")
        time.sleep(1.0)  # 안내 읽고 준비할 시간
        rest_samples = _collect(reader, seconds)
        rest_baseline = int(statistics.median(rest_samples))
        rest_std = statistics.pstdev(rest_samples) if len(rest_samples) > 1 else 0.0
        print(f"[calibrate] REST 중앙값={rest_baseline}, 표준편차={rest_std:.0f}")

        print(f"\n[calibrate] 물건을 집듯이 편하게 쥐고 {seconds:.0f}초 유지하세요...")
        time.sleep(1.0)
        ref_samples = _collect(reader, seconds)
        ref_contraction = int(statistics.median(ref_samples))
        print(f"[calibrate] 편한 쥐기 중앙값={ref_contraction} (순간 최고={max(ref_samples)})")

        separation = (ref_contraction - rest_baseline) / max(rest_std, 1)
        if ref_contraction > rest_baseline and separation >= MIN_SEPARATION_STD:
            break
        print(
            f"[calibrate] 경고: 쥐기 상승폭이 rest 잡음의 {separation:.1f}배뿐입니다 (최소 {MIN_SEPARATION_STD}배). "
            "전극 접촉(REF 위치, 스냅, 젤 마름)을 확인하세요."
        )
        if attempt <= retries:
            print(f"[calibrate] 다시 시도합니다 ({attempt}/{retries})")
    else:
        print("[calibrate] 경고: 재시도 후에도 차이가 작습니다. 이 값으로 진행하지만 판정이 불안정할 수 있습니다.")

    print(f"[calibrate] 완료: rest_baseline={rest_baseline}, ref_contraction={ref_contraction}\n")
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
