"""
기존(최대 수축 보정) vs 제안(편한 쥐기 보정) 파이프라인 재생 비교.

논문 "문제 2" 근거: 최대 수축 기준으로 보정하면 편하게 쥐었을 때 로봇손이 완전히 닫히지 않고
Hold(반쯤) 상태에 머물며 명령이 계속 뒤집힌다(2026-09-21 run.log 실측). 같은 녹화를
두 파이프라인에 넣어 그 현상을 숫자로 재현한다.

기존 파이프라인 = git 커밋 40b6ecd (2026-09-21 보정 수정 직전) 의 emg_pipeline/:
  - 보정: rest_baseline = rest 평균 + 1σ, max_contraction = 최대 쥐기 최고치 x 0.9
  - 판정: 평활 alpha 0.15, 임계 LIGHT 0.1 / STRONG 0.4 / GRIP 0.7, 히스테리시스 없음
  - 명령: REST→Release, LIGHT→Hold, STRONG/GRIP→GripClose
제안 파이프라인 = 현재 emg_pipeline/:
  - 보정: rest 중앙값, ref_contraction = 편한 쥐기 앞 2회 중앙값
  - 판정: 평활 alpha 0.02, 임계 0.2 / 0.65 / 0.7, 해제 비율 0.43 히스테리시스
  - 명령: GripLatch (STRONG/GRIP→GripClose, REST→Release, LIGHT→이전 유지)

평가 구간: light 파일이 있으면 light 3~6회(보정에 안 쓴 회차), 없으면 grip 3~6회.
기존 보정의 '최대'는 grip 파일 앞 2회 최고치, 제안 보정의 '편한 쥐기'는 light(없으면 grip) 앞 2회 중앙값.
rest 오탐은 rest 전체(전환 0.5초 제외)에서 GripClose 비율.

사용법 (저장소 루트에서):
  python3 analysis/replay_old_vs_new.py                    # 아래 SUBJECTS 전부
  python3 analysis/replay_old_vs_new.py n1 n6              # 지정 피험자만
  python3 analysis/replay_old_vs_new.py --md analysis/out/replay_old_vs_new.md
"""

import argparse
import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "analysis"))

import compare_thresholds as ct                                  # noqa: E402  (load, DATA_DIR)
from emg_pipeline.serial_reader import EMGSerialReader as NewReader  # noqa: E402
from emg_pipeline.command_mapper import GripLatch                # noqa: E402

OLD_COMMIT = "40b6ecd"
FS = 500

# (이름, rest, grip(최대), light(편한 쥐기) 또는 None)
SUBJECTS = [
    ("s1_v5", "s1_rest_v5.csv", "s1_grip_v5.csv", None),
    ("s1_v6", "s1_rest_v6.csv", "s1_grip_v6.csv", None),
    ("n1", "n1_rest.csv", "n1_grip.csv", "n1_light.csv"),
    ("n2", "n2_rest.csv", "n2_grip.csv", "n2_light.csv"),
    ("n3", "n3_rest.csv", "n3_grip.csv", "n3_light.csv"),
    ("n4", "n4_rest.csv", "n4_grip.csv", "n4_light.csv"),
    ("n5", "n5_rest.csv", "n5_grip.csv", "n5_light.csv"),
    ("n6", "n6_rest.csv", "n6_grip.csv", "n6_light.csv"),
]


def load_old_pipeline():
    """git 이력에서 기존 emg_pipeline 을 임시 폴더로 꺼내 import 한다."""
    tmp = tempfile.mkdtemp(prefix="oldpipe_")
    mods = {}
    for name in ("serial_reader", "command_mapper"):
        src = subprocess.check_output(["git", "-C", ROOT, "show", f"{OLD_COMMIT}:emg_pipeline/{name}.py"])
        path = os.path.join(tmp, f"old_{name}.py")
        with open(path, "wb") as f:
            f.write(src)
        spec = importlib.util.spec_from_file_location(f"old_{name}", path)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        mods[name] = m
    return mods["serial_reader"], mods["command_mapper"]


def run_old(seq, rest_base, max_c, old_sr, old_cm):
    rd = old_sr.EMGSerialReader("replay", rest_baseline=rest_base, max_contraction=max_c)
    out = []
    for v in seq:
        rd._handle_line(f"0,{v}")
        out.append(old_cm.map_to_robot_command(rd.current_state).value)
    return np.array(out)


def run_new(seq, rest_base, ref):
    rd = NewReader("replay", rest_baseline=rest_base, ref_contraction=ref)
    latch = GripLatch()
    out = []
    for v in seq:
        rd._handle_line(f"0,{v}")
        out.append(latch.update(rd.current_state).value)
    return np.array(out)


def metrics(cmds, n_rest_head, n_eval):
    ev = cmds[n_rest_head:n_rest_head + n_eval]
    rest = np.concatenate([cmds[:n_rest_head], cmds[n_rest_head + n_eval + FS // 2:]])
    return dict(
        close=(ev == "GripClose").mean() * 100,
        hold=(ev == "Hold").mean() * 100,
        switches=int((ev[1:] != ev[:-1]).sum()),
        fp=(rest == "GripClose").mean() * 100,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("subjects", nargs="*", help="예: n1 n6 (없으면 전부)")
    p.add_argument("--md", help="markdown 표 저장 경로")
    args = p.parse_args()

    old_sr, old_cm = load_old_pipeline()
    rows = []
    for name, restf, gripf, lightf in SUBJECTS:
        if args.subjects and name not in args.subjects:
            continue
        rest, _ = ct.load(os.path.join(ct.DATA_DIR, restf))
        _, greps = ct.load(os.path.join(ct.DATA_DIR, gripf))
        lreps = ct.load(os.path.join(ct.DATA_DIR, lightf))[1] if lightf else greps
        ev = np.concatenate(lreps[2:])
        half = len(rest) // 2
        seq = np.concatenate([rest[:half], ev, rest[half:]])

        o_rest = int(rest.mean() + rest.std())
        o_max = int(0.9 * max(np.concatenate(greps[:2])))
        n_rest = int(np.median(rest))
        n_ref = int(np.median(np.concatenate(lreps[:2])))

        mo = metrics(run_old(seq, o_rest, o_max, old_sr, old_cm), half, len(ev))
        mn = metrics(run_new(seq, n_rest, n_ref), half, len(ev))
        rows.append(dict(name=name, src="light 3~6회" if lightf else "grip 3~6회",
                         o_rest=o_rest, o_max=o_max, n_rest=n_rest, n_ref=n_ref, old=mo, new=mn))

    hdr = "| 피험자 | 평가 구간 | 기존 rest/max | 기존 닫힘% | 기존 Hold% | 기존 전환 | 기존 rest오탐% | 제안 rest/ref | 제안 닫힘% | 제안 Hold% | 제안 전환 | 제안 rest오탐% |"
    sep = "|---|---|---|---|---|---|---|---|---|---|---|---|"
    lines = [hdr, sep]
    for r in rows:
        o, n = r["old"], r["new"]
        lines.append(f"| {r['name']} | {r['src']} | {r['o_rest']}/{r['o_max']} | {o['close']:.1f} | {o['hold']:.1f} | {o['switches']} | {o['fp']:.1f} "
                     f"| {r['n_rest']}/{r['n_ref']} | {n['close']:.1f} | {n['hold']:.1f} | {n['switches']} | {n['fp']:.1f} |")
    table = "\n".join(lines)
    print(f"기존 파이프라인 = git {OLD_COMMIT}, 제안 = 현재 emg_pipeline/\n")
    print(table)
    if args.md:
        os.makedirs(os.path.dirname(args.md) or ".", exist_ok=True)
        with open(args.md, "w") as f:
            f.write(f"기존 파이프라인 = git {OLD_COMMIT}, 제안 = 현재 emg_pipeline/. 평가 = 편하게 쥔 회차(보정에 안 쓴 3~6회) + rest 전체.\n\n{table}\n")
        print(f"\n표 저장: {args.md}")


if __name__ == "__main__":
    main()
