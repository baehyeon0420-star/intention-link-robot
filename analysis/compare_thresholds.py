"""
쥐기 판정 기준 비교 (오프라인). collect.py가 만든 피험자별 rest / light / (선택) grip csv를 읽어서
세 방식의 판정 성능을 k(=grip_on) 값별로 비교하고, 논문에 붙일 markdown 표와 그래프를 만든다.

  A) 고정 임계값      : 모든 사람에게 rest 18 / ref 1321 (예전 기본값) 고정
  B) 편한 쥐기 보정   : 개인별 rest 중앙값 + light 앞 2회 중앙값을 ref 로 → k
  C) 최대 수축 보정   : 개인별 rest 중앙값 + grip 앞 2회 중앙값을 ref 로 → 같은 k (grip 있는 사람만)

평가는 light 뒤 4회(보정에 안 쓴 회차)와 rest 전체로 한다. light 가 없는 피험자는 B 를 못 하고,
C 도 grip 뒤 4회로 평가한다 (표에 '평가=grip' 으로 표시).

판정은 실제 로봇 경로 그대로: serial_reader (EMA 평활, threshold_strong=k, release_ratio 0.43 히스테리시스)
→ command_mapper.GripLatch (STRONG/GRIP→GripClose, REST→Release, LIGHT→이전 유지).
회차 분리는 collect.py 의 t_ms 간격(회차 사이 휴식으로 생기는 공백)으로 자른다.

사용법 (저장소 루트에서):
  python3 analysis/compare_thresholds.py                        # emg_data_collection/*.csv 전부
  python3 analysis/compare_thresholds.py --subjects s5 s6 --k 0.7
  python3 analysis/compare_thresholds.py --out analysis/out     # 표(md/csv)와 그래프 저장 위치
"""

import argparse
import glob
import os
import re
import statistics
import sys

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from emg_pipeline.command_mapper import GripLatch, RobotCommand  # noqa: E402
from emg_pipeline.serial_reader import EMGSerialReader  # noqa: E402

DATA_DIR = os.path.join(ROOT, "emg_data_collection")
FS = 500
REPS = 6
CAL_REPS = 2
RELEASE_RATIO = 0.43           # serial_reader 기본값
GAP_MS = 1000                  # t_ms 가 이만큼 이상 벌어지면 회차 경계
FIXED = dict(rest=18, ref=1321)  # A) 예전 기본값
KS = np.round(np.arange(0.3, 0.91, 0.05), 2)


def load(path):
    """(raw 배열, 회차 리스트). 회차는 t_ms 공백으로 나눈다. 공백이 없으면 REPS 등분."""
    df = pd.read_csv(path)
    raw = pd.to_numeric(df["raw"], errors="coerce")
    t = pd.to_numeric(df["t_ms"], errors="coerce") if "t_ms" in df else None
    ok = raw.notna() & (raw >= 0) & (raw <= 4095)
    v = raw[ok].astype(int).values
    if t is not None and t[ok].notna().all():
        tt = t[ok].values
        cut = np.where(np.diff(tt) > GAP_MS)[0] + 1
        reps = np.split(v, cut) if len(cut) else [v]
    else:
        reps = [v]
    return v, reps


def split_reps(x, reps=REPS):
    return np.array_split(x, reps)


def drop_weak_reps(chunks, min_ratio, label, name):
    """회차 평균이 최대 회차의 min_ratio 미만이면 접촉 불량으로 제외 (calibration_compare.py와 같은 규칙)."""
    if chunks is None or min_ratio <= 0:
        return chunks
    means = [c.mean() for c in chunks]
    keep = [i for i, m in enumerate(means) if m >= min_ratio * max(means)]
    if len(keep) < len(chunks):
        print(f"[{name}] {label} 제외 회차: {[i + 1 for i in range(len(chunks)) if i not in keep]} (최대 회차의 {min_ratio:.0%} 미만)")
    return [chunks[i] for i in keep]


def find_subjects(names):
    files = glob.glob(os.path.join(DATA_DIR, "*_rest*.csv"))
    subs = {}
    for f in sorted(files):
        base = os.path.basename(f)[:-4]
        name = re.sub(r"_rest", "", base, count=1)
        if names and name not in names:
            continue
        d = {"rest": f}
        for g in ("light", "grip"):
            p = f.replace("_rest", f"_{g}", 1)
            if os.path.exists(p):
                d[g] = p
        subs[name] = d
    return subs


def replay_latch(seq, rest, ref, k):
    rd = EMGSerialReader("replay", rest_baseline=rest, ref_contraction=ref, threshold_strong=k, release_ratio=RELEASE_RATIO)
    latch = GripLatch()
    on = np.empty(len(seq), dtype=bool)
    for i, v in enumerate(seq):
        rd._handle_line(f"0,{v}")
        on[i] = latch.update(rd.current_state) == RobotCommand.GRIP_CLOSE
    return on


def evaluate(rest_all, eval_grip, rest_base, ref, k):
    """(쥐기 구간 판정률 %, rest 오탐률 %). rest → eval → rest 순서로 흘려서 래치 동작까지 포함."""
    half = len(rest_all) // 2
    seq = np.concatenate([rest_all[:half], eval_grip, rest_all[half:]])
    on = replay_latch(seq, rest_base, ref, k)
    g = on[half: half + len(eval_grip)]
    r = np.concatenate([on[:half], on[half + len(eval_grip) + FS // 2:]])   # 힘 빼는 전환 0.5초 제외
    return 100 * g.mean(), 100 * r.mean()


def main():
    p = argparse.ArgumentParser(description="쥐기 판정 기준 비교 (A 고정 / B 편한 쥐기 / C 최대 수축)")
    p.add_argument("--subjects", nargs="*")
    p.add_argument("--k", type=float, default=0.7, help="요약 표에 쓸 grip_on 값")
    p.add_argument("--out", default=os.path.join(ROOT, "analysis", "out"))
    p.add_argument("--min-rep-ratio", type=float, default=0.0,
                   help="회차 평균이 최대 회차의 이 비율 미만이면 접촉 불량으로 제외 (예 0.4). 모든 피험자에 같은 값을 쓸 것")
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)

    subs = find_subjects(args.subjects)
    if not subs:
        print("rest csv 를 못 찾았습니다."); return

    rows = []
    for name, files in subs.items():
        rest, _ = load(files["rest"])
        rest_base = int(np.median(rest))
        light_reps = load(files["light"])[1] if "light" in files else None
        grip_reps = load(files["grip"])[1] if "grip" in files else None
        if light_reps is None and grip_reps is None:
            print(f"[{name}] light/grip 없음 — 건너뜀"); continue
        light_reps = drop_weak_reps(light_reps, args.min_rep_ratio, "light", name)
        grip_reps = drop_weak_reps(grip_reps, args.min_rep_ratio, "grip", name)

        # 평가 구간: light 앞 2회 보정, 나머지 평가 (없으면 grip으로). 남은 회차가 3개 미만이면 건너뜀
        if light_reps is not None and len(light_reps) >= CAL_REPS + 1:
            eval_seg = np.concatenate(light_reps[CAL_REPS:]); eval_src = f"light({len(light_reps)}회)"
            ref_b = int(np.median(np.concatenate(light_reps[:CAL_REPS])))
        elif grip_reps is not None and len(grip_reps) >= CAL_REPS + 1:
            eval_seg = np.concatenate(grip_reps[CAL_REPS:]); eval_src = f"grip({len(grip_reps)}회)"
            ref_b = None
        else:
            print(f"[{name}] 유효 회차 부족 — 건너뜀"); continue
        ref_c = int(np.median(np.concatenate(grip_reps[:CAL_REPS]))) if grip_reps is not None and len(grip_reps) >= CAL_REPS else None

        pct_mvc = None
        if light_reps is not None and grip_reps is not None:
            pct_mvc = 100 * (np.median(np.concatenate(light_reps)) - rest_base) / max(np.median(np.concatenate(grip_reps)) - rest_base, 1)

        for k in KS:
            row = dict(subject=name, k=k, eval=eval_src, rest_median=rest_base, ref_B=ref_b, ref_C=ref_c, light_pct_mvc=pct_mvc)
            row["A_hit"], row["A_fp"] = evaluate(rest, eval_seg, FIXED["rest"], FIXED["ref"], k)
            if ref_b is not None:
                row["B_hit"], row["B_fp"] = evaluate(rest, eval_seg, rest_base, ref_b, k)
            if ref_c is not None:
                row["C_hit"], row["C_fp"] = evaluate(rest, eval_seg, rest_base, ref_c, k)
            rows.append(row)

    df = pd.DataFrame(rows)
    df.round(2).to_csv(os.path.join(args.out, "compare_thresholds_all_k.csv"), index=False)

    # ── k 스캔 요약 (피험자 평균) ──
    methods = [m for m in "ABC" if f"{m}_hit" in df]
    scan = df.groupby("k")[[f"{m}_{x}" for m in methods for x in ("hit", "fp")]].mean().round(1)
    print("\n■ k별 평균 (판정률% / rest 오탐률%)")
    print(scan.to_string())

    # ── 지정 k 요약 표 (markdown) ──
    sel = df[np.isclose(df.k, args.k)]
    md = [f"| 피험자 | 평가 구간 | rest 중앙값 | ref(B 편한쥐기) | ref(C 최대) | light %MVC | " +
          " | ".join(f"{m} 판정% | {m} 오탐%" for m in methods) + " |",
          "|" + "---|" * (6 + 2 * len(methods))]
    for _, r in sel.iterrows():
        cells = [r.subject, r.eval, f"{r.rest_median:.0f}",
                 "-" if pd.isna(r.ref_B) else f"{r.ref_B:.0f}", "-" if pd.isna(r.ref_C) else f"{r.ref_C:.0f}",
                 "-" if pd.isna(r.light_pct_mvc) else f"{r.light_pct_mvc:.0f}%"]
        for m in methods:
            cells += ["-" if pd.isna(r.get(f"{m}_hit")) else f"{r[f'{m}_hit']:.1f}",
                      "-" if pd.isna(r.get(f"{m}_fp")) else f"{r[f'{m}_fp']:.1f}"]
        md.append("| " + " | ".join(cells) + " |")
    mean_cells = ["**평균**", "", "", "", "", ""]
    for m in methods:
        mean_cells += [f"**{sel[f'{m}_hit'].mean():.1f}**", f"**{sel[f'{m}_fp'].mean():.1f}**"]
    md.append("| " + " | ".join(mean_cells) + " |")
    md_text = f"k = threshold_strong = {args.k} (해제 {args.k * RELEASE_RATIO:.2f}, LIGHT 는 이전 유지), n = {sel.subject.nunique()}\n\n" + "\n".join(md)
    print("\n■ 요약 표 (k=%.2f)\n" % args.k + md_text)
    with open(os.path.join(args.out, f"compare_thresholds_k{args.k:.2f}.md"), "w", encoding="utf-8") as f:
        f.write(md_text + "\n")

    # ── 그래프 ──
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 4))
        for m, c in zip(methods, ("gray", "tab:blue", "tab:orange")):
            ax[0].plot(scan.index, scan[f"{m}_hit"], "-o", color=c, label=m)
            ax[1].plot(scan.index, scan[f"{m}_fp"], "-o", color=c, label=m)
        ax[0].set_title("grip detection rate (%)"); ax[1].set_title("rest false-positive rate (%)")
        for a in ax:
            a.set_xlabel("k (threshold_strong)"); a.grid(alpha=.3); a.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "compare_thresholds.png"), dpi=150)
        print(f"\n그래프 저장: {os.path.join(args.out, 'compare_thresholds.png')}")
    except Exception as e:
        print(f"\n그래프 생략 ({e!r})")
    print(f"표 저장: {args.out}/compare_thresholds_k{args.k:.2f}.md, compare_thresholds_all_k.csv")


if __name__ == "__main__":
    main()
