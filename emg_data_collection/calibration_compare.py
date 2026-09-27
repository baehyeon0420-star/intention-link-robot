"""
보정 방식 비교: 피험자별 rest/grip 녹화 데이터에 두 보정 방식을 되돌려서(replay) 판정 성능을 표로 뽑는다.

  예전 방식 : max_contraction = 3초 쥐기 중 순간 최고치 x 0.9, 평활 0.15, 임계 0.1/0.4/0.7, 히스테리시스 없음
  새 방식   : max_contraction = 3초 쥐기 중앙값, 평활 0.02, 임계 0.2/0.35/0.7, 히스테리시스(x0.43)
              (emg_pipeline/serial_reader.py, calibration.py 기본값 — 2026-09-21 커밋 8bdd7a3)

보정은 실제 auto_calibrate와 같은 조건으로 흉내낸다: rest 첫 3초, grip 첫 3초를 20ms 간격(50Hz)으로 150샘플.
그 뒤 rest → grip → rest 순서로 500Hz 원시 데이터를 EMGSerialReader에 그대로 흘려서 상태를 기록한다.

사용법 (수집 폴더에서):
  python3 calibration_compare.py                 # 이 폴더의 *_rest*.csv / *_grip*.csv 쌍 전부
  python3 calibration_compare.py s5 s6 s7        # 지정 피험자만 (s5_rest.csv, s5_grip.csv)
  python3 calibration_compare.py --csv out.csv   # 표를 CSV로도 저장

출력 지표
  쥐기 판정률 : grip 구간에서 GripClose(STRONG/GRIP) 명령이 나온 시간 비율
  필요 힘     : 쥐기 판정이 시작되는 raw 값이 grip 중앙값의 몇 %인지 (낮을수록 약하게 쥐어도 됨)
  rest 오작동 : rest 구간에서 REST가 아닌 상태가 나온 시간 비율
  깜빡임      : grip 구간에서 쥐기 명령이 켜졌다 꺼진 횟수
  분리도      : (grip 중앙값 - rest 평균) / rest 표준편차 (sensor_check.py 기준, 3 미만이면 신호 불량)
"""

import argparse
import glob
import os
import re
import statistics
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from emg_pipeline.serial_reader import EMGSerialReader  # noqa: E402

METHODS = {
    # 이름: (max_contraction 계산, reader 인자)
    "old": (lambda g: int(max(g) * 0.9),
            dict(threshold_light=0.1, threshold_strong=0.4, threshold_grip=0.7, release_ratio=1.0, smoothing_alpha=0.15)),
    "new": (lambda g: int(statistics.median(g)),
            dict(threshold_light=0.2, threshold_strong=0.35, threshold_grip=0.7, release_ratio=0.43, smoothing_alpha=0.02)),
            # 커밋 8bdd7a3 기준값을 고정 (이후 serial_reader 기본값이 바뀌어도 비교가 흔들리지 않게)
}
CAL_SAMPLES = 150   # auto_calibrate: 3초 x 50Hz
FS = 500            # 펌웨어 샘플링


def load(path):
    v = pd.to_numeric(pd.read_csv(path)["raw"], errors="coerce").dropna().astype(int).values
    return v[(v >= 0) & (v <= 4095)]


def find_pairs(names):
    pairs = {}
    if names:
        for n in names:
            r, g = f"{n}_rest.csv", f"{n}_grip.csv"
            if os.path.exists(r) and os.path.exists(g):
                pairs[n] = (r, g)
            else:
                print(f"[{n}] {r} / {g} 없음 — 건너뜀")
    else:
        for r in sorted(glob.glob("*_rest*.csv")):
            g = r.replace("_rest", "_grip", 1)
            if os.path.exists(g):
                pairs[re.sub(r"_rest", "", r[:-4], count=1)] = (r, g)
    return pairs


def exclude_weak_reps(grip, reps, min_ratio):
    """grip을 회차 수로 등분해 평균이 최대 회차의 min_ratio 미만인 회차를 뺀다. (뺀 회차 번호 반환, 1부터)"""
    if min_ratio <= 0 or reps <= 1:
        return grip, []
    chunks = np.array_split(grip, reps)
    means = [c.mean() for c in chunks]
    keep = [i for i, m in enumerate(means) if m >= min_ratio * max(means)]
    dropped = [i + 1 for i in range(reps) if i not in keep]
    return np.concatenate([chunks[i] for i in keep]), dropped


def exclude_rest_windows(rest, max_mean):
    """rest를 1초 구간으로 나눠 평균이 max_mean을 넘는 구간을 뺀다."""
    if max_mean <= 0:
        return rest, []
    chunks = [rest[i:i + FS] for i in range(0, len(rest), FS)]
    keep = [c for c in chunks if c.mean() <= max_mean]
    dropped = [i for i, c in enumerate(chunks) if c.mean() > max_mean]
    return np.concatenate(keep) if keep else rest[:0], dropped


def calib_subsample(x):
    """auto_calibrate처럼 3초 동안 20ms마다 current_value를 읽은 것을 흉내: 첫 1500샘플에서 10개마다."""
    return x[: FS * 3 : FS // 50][:CAL_SAMPLES].tolist()


def replay(rest, grip, method):
    mc_fn, kw = METHODS[method]
    cal_r, cal_g = calib_subsample(rest), calib_subsample(grip)
    rb = int(statistics.mean(cal_r) + statistics.pstdev(cal_r))
    mc = mc_fn(cal_g)

    rd = EMGSerialReader("replay", rest_baseline=rb, max_contraction=mc, **kw)
    half = len(rest) // 2
    seq = np.concatenate([rest[:half], grip, rest[half:]])
    states = []
    for v in seq:
        rd._handle_line(f"0,{v}")
        states.append(rd.current_state)
    st = np.array(states)
    g = st[half: half + len(grip)]
    # grip 직후 rest 0.5초는 힘 빼는 전환 구간이라 rest 오작동에서 제외
    r = np.concatenate([st[:half], st[half + len(grip) + FS // 2:]])

    close = np.isin(g, ["STRONG", "GRIP"])
    flicker = int(np.abs(np.diff(close.astype(int))).sum())
    thr_raw = rb + rd.threshold_strong * (mc - rb)
    return dict(
        rb=rb, mc=mc,
        close_pct=100 * close.mean(),
        effort_pct=100 * (thr_raw - statistics.mean(cal_r)) / max(statistics.median(cal_g) - statistics.mean(cal_r), 1),
        rest_err_pct=100 * (r != "REST").mean(),
        flicker=flicker,
    )


def main():
    p = argparse.ArgumentParser(description="보정 방식 비교 (예전: 피크x0.9 / 새: 중앙값+히스테리시스)")
    p.add_argument("subjects", nargs="*", help="예: s5 s6 (없으면 폴더의 *_rest*.csv 전부)")
    p.add_argument("--csv", help="결과 CSV 저장 경로")
    p.add_argument("--reps", type=int, default=6, help="grip 파일의 회차 수 (collect.py --reps)")
    p.add_argument("--min-rep-ratio", type=float, default=0.0,
                   help="회차 평균이 최대 회차 평균의 이 비율 미만이면 접촉 불량으로 제외 (예 0.4). 0이면 제외 안 함")
    p.add_argument("--rest-window-max", type=float, default=0.0,
                   help="rest 1초 구간 평균이 이 값을 넘으면 접촉 불량으로 제외. 0이면 제외 안 함")
    args = p.parse_args()

    pairs = find_pairs(args.subjects)
    if not pairs:
        print("rest/grip csv 쌍을 못 찾았습니다.")
        return

    rows = []
    for name, (rf, gf) in pairs.items():
        rest, grip = load(rf), load(gf)
        grip, dropped = exclude_weak_reps(grip, args.reps, args.min_rep_ratio)
        if dropped:
            print(f"[{name}] 제외한 회차: {dropped} (최대 회차 평균의 {args.min_rep_ratio:.0%} 미만)")
        rest, dropped_w = exclude_rest_windows(rest, args.rest_window_max)
        if dropped_w:
            print(f"[{name}] 제외한 rest 구간(초): {dropped_w} (평균 > {args.rest_window_max:.0f})")
        if len(rest) < FS * 3 or len(grip) < FS * 3:
            print(f"[{name}] 데이터 부족 (rest {len(rest)}, grip {len(grip)} 샘플) — 건너뜀")
            continue
        sep = (statistics.median(grip) - rest.mean()) / max(rest.std(), 1)
        row = dict(subject=name, rest_mean=rest.mean(), rest_std=rest.std(), grip_median=float(np.median(grip)), separation=sep)
        for m in METHODS:
            for k, v in replay(rest, grip, m).items():
                row[f"{m}_{k}"] = v
        rows.append(row)

    df = pd.DataFrame(rows)
    pd.set_option("display.width", 200)
    print("\n■ 피험자별 신호")
    print(df[["subject", "rest_mean", "rest_std", "grip_median", "separation"]].round(1).to_string(index=False))
    for m, label in (("old", "예전 방식 (피크x0.9, 히스테리시스 없음)"), ("new", "새 방식 (중앙값, 히스테리시스)")):
        print(f"\n■ {label}")
        cols = {f"{m}_mc": "max_contraction", f"{m}_close_pct": "쥐기판정률%", f"{m}_effort_pct": "필요힘%(grip중앙값 대비)",
                f"{m}_rest_err_pct": "rest오작동%", f"{m}_flicker": "깜빡임"}
        print(df[["subject"] + list(cols)].rename(columns=cols).round(1).to_string(index=False))

    print("\n■ 평균 (n=%d)" % len(df))
    for k, lab in (("close_pct", "쥐기 판정률%"), ("effort_pct", "필요 힘%"), ("rest_err_pct", "rest 오작동%"), ("flicker", "깜빡임 횟수")):
        print(f"  {lab:12s} 예전 {df[f'old_{k}'].mean():6.1f} ± {df[f'old_{k}'].std(ddof=0):5.1f}   새 {df[f'new_{k}'].mean():6.1f} ± {df[f'new_{k}'].std(ddof=0):5.1f}")
    low = df[df.separation < 3].subject.tolist()
    if low:
        print(f"\n⚠️ 분리도 3 미만(신호 불량 의심): {low}")

    if args.csv:
        df.round(2).to_csv(args.csv, index=False)
        print(f"\n저장: {args.csv}")


if __name__ == "__main__":
    main()
