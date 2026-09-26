"""
그림 1: 같은 편한 쥐기 녹화를 기존/제안 파이프라인에 넣었을 때의 명령 타임라인.

위: 원신호(0·4095 제외 후), 아래 두 줄: 기존(최대 수축 보정, git 40b6ecd)과 제안(현재 emg_pipeline)의 로봇 명령.
평가 구간은 replay_old_vs_new.py 와 동일 (rest 앞 절반 → 편한 쥐기 3~6회 → rest 뒤 절반).

사용법 (저장소 루트에서):
  python3 analysis/fig_command_timeline.py n7 --out analysis/out/fig1_timeline_n7.png
"""
import argparse, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "analysis"))
import compare_thresholds as ct
import replay_old_vs_new as ro

FS = 500
CMD_LEVEL = {"Release": 0, "Hold": 1, "GripClose": 2}


def pick_font():
    for name in ["KoPubWorld Batang Light", "KoPubWorldBatang Light", "AppleGothic", "Apple SD Gothic Neo", "NanumGothic"]:
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            return name
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("subject", nargs="?", default="n7")
    p.add_argument("--out", default=None)
    a = p.parse_args()
    D = ct.DATA_DIR
    rest, _ = ct.load(f"{D}/{a.subject}_rest.csv")
    _, greps = ct.load(f"{D}/{a.subject}_grip.csv")
    _, lreps = ct.load(f"{D}/{a.subject}_light.csv")
    ev = np.concatenate(lreps[2:]); half = len(rest) // 2
    seq = np.concatenate([rest[:half], ev, rest[half:]])
    t = np.arange(len(seq)) / FS

    o_rest = int(rest.mean() + rest.std()); o_max = int(0.9 * max(np.concatenate(greps[:2])))
    n_rest = int(np.median(rest)); n_ref = int(np.median(np.concatenate(lreps[:2])))
    old_sr, old_cm = ro.load_old_pipeline()
    c_old = ro.run_old(seq, o_rest, o_max, old_sr, old_cm)
    c_new = ro.run_new(seq, n_rest, n_ref)
    y_old = np.array([CMD_LEVEL[c] for c in c_old]); y_new = np.array([CMD_LEVEL[c] for c in c_new])

    font = pick_font()
    if font: plt.rcParams["font.family"] = font
    plt.rcParams["axes.unicode_minus"] = False
    # 85mm 단 폭에 원본 크기로 들어가도록 3.35in 폭, 글자 8pt 이상
    plt.rcParams.update({"font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "xtick.labelsize": 7.5, "ytick.labelsize": 7.5})
    fig, ax = plt.subplots(3, 1, figsize=(3.35, 2.7), sharex=True, gridspec_kw={"height_ratios": [1.6, 1, 1]})
    ax[0].plot(t, seq, lw=0.4, color="k")
    ax[0].axvspan(half / FS, (half + len(ev)) / FS, color="#dddddd", alpha=0.6)
    ax[0].set_ylabel("sEMG (ADC)")
    ax[0].set_title(f"{a.subject}: rest → 준최대 수축 4회(회색) → rest")
    ev_slice = slice(half, half + len(ev))
    for axis, y, label in [(ax[1], y_old, "기존"), (ax[2], y_new, "제안")]:
        axis.step(t, y, where="post", lw=0.7, color="k")
        axis.set_yticks([0, 1, 2]); axis.set_yticklabels(["열림", "반쯤", "닫힘"])
        axis.set_ylim(-0.3, 2.9); axis.set_ylabel(label)
        ye = y[ev_slice]
        sw = int((ye[1:] != ye[:-1]).sum())            # 표 2와 동일: 평가 구간 안의 명령 전환만
        closed = (ye == 2).mean() * 100
        axis.text(0.01, 0.97, f"수축 구간 닫힘 {closed:.1f}%, 전환 {sw}회", ha="left", va="top",
                  transform=axis.transAxes, fontsize=8, bbox=dict(fc="white", ec="none", alpha=0.85, pad=1))
    ax[2].set_xlabel("time (s)")
    fig.tight_layout(pad=0.3)
    out = a.out or os.path.join(ROOT, "analysis", "out", f"fig1_timeline_{a.subject}.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=300)
    print("저장:", out)


if __name__ == "__main__":
    main()
