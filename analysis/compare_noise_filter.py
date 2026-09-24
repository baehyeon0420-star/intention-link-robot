"""
노이즈(0·4095) 포함 vs 제외 비교 — 논문 "문제 3" 근거.

collect.py 는 0(ADC 바닥 = 접촉 끊김)과 4095(포화) 를 뺀 본 CSV 와, 필터 전 전체 샘플(raw/) 을 같이 남긴다.
같은 피험자를 두 버전으로 compare_thresholds 와 같은 방식(rest 중앙값 + 앞 2회 중앙값 기준, k, 히스테리시스·래치 재생)
으로 판정해서, 평균 통계는 얼마나 흔들리고 중앙값 기반 판정은 얼마나 흔들리는지 나란히 놓는다.

사용법 (저장소 루트에서):
  python3 analysis/compare_noise_filter.py                 # n1 n6 n7
  python3 analysis/compare_noise_filter.py n2 --k 0.7 --md analysis/out/compare_noise_filter.md
"""

import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "analysis"))
import compare_thresholds as ct  # noqa: E402

DEFAULT_SUBJECTS = ["n1", "n2", "n3", "n4"]


def load_rest_raw(path):
    """raw/ rest 는 여러 번 실행분이 이어붙어 있을 수 있어 마지막 실행분만 쓴다 (본 CSV 와 같은 구간)."""
    v, reps = ct.load(path)
    return reps[-1]


def stats_and_eval(rest, light_reps, grip_reps, k):
    rest_base = int(np.median(rest))
    ref_b = int(np.median(np.concatenate(light_reps[:2])))
    ref_c = int(np.median(np.concatenate(grip_reps[:2])))
    ev = np.concatenate(light_reps[2:])
    b_hit, b_fp = ct.evaluate(rest, ev, rest_base, ref_b, k)
    c_hit, c_fp = ct.evaluate(rest, ev, rest_base, ref_c, k)
    return dict(rest_mean=rest.mean(), rest_std=rest.std(), rest_med=rest_base, ref_b=ref_b, ref_c=ref_c,
                zero_pct=(rest == 0).mean() * 100, sat_pct=(rest >= 4095).mean() * 100,
                b_hit=b_hit, b_fp=b_fp, c_hit=c_hit, c_fp=c_fp)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("subjects", nargs="*", default=DEFAULT_SUBJECTS)
    p.add_argument("--k", type=float, default=0.7)
    p.add_argument("--md")
    args = p.parse_args()
    D = ct.DATA_DIR
    rows = []
    for s in args.subjects:
        filt = dict(rest=ct.load(f"{D}/{s}_rest.csv")[0], light=ct.load(f"{D}/{s}_light.csv")[1], grip=ct.load(f"{D}/{s}_grip.csv")[1])
        raw = dict(rest=load_rest_raw(f"{D}/raw/{s}_rest_raw.csv"), light=ct.load(f"{D}/raw/{s}_light_raw.csv")[1], grip=ct.load(f"{D}/raw/{s}_grip_raw.csv")[1])
        rows.append((s, stats_and_eval(raw["rest"], raw["light"], raw["grip"], args.k), stats_and_eval(filt["rest"], filt["light"], filt["grip"], args.k)))

    hdr = ("| 피험자 | 조건 | rest 평균 | rest 표준편차 | rest 중앙값 | 0값% | 4095% | B 기준 | C 기준 | B 판정% | B 오탐% | C 판정% | C 오탐% |\n"
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    lines = [hdr]
    for s, r, f in rows:
        for label, m in (("노이즈 포함(raw)", r), ("노이즈 제외", f)):
            lines.append(f"| {s} | {label} | {m['rest_mean']:.0f} | {m['rest_std']:.0f} | {m['rest_med']} | {m['zero_pct']:.0f} | {m['sat_pct']:.1f} "
                         f"| {m['ref_b']} | {m['ref_c']} | {m['b_hit']:.1f} | {m['b_fp']:.1f} | {m['c_hit']:.1f} | {m['c_fp']:.1f} |")
    table = "\n".join(lines)
    print(f"k = {args.k}. 판정 = rest 중앙값 + 앞 2회 중앙값 기준, 평가 = light 3~6회 + rest 전체\n\n{table}")
    if args.md:
        os.makedirs(os.path.dirname(args.md) or ".", exist_ok=True)
        with open(args.md, "w") as fh:
            fh.write(f"k = {args.k}. 판정 = rest 중앙값 + 앞 2회 중앙값 기준, 평가 = light 3~6회 + rest 전체\n\n{table}\n")
        print(f"\n표 저장: {args.md}")


if __name__ == "__main__":
    main()
