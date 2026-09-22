"""
EMG 원시 신호 수집.

개선된 프로토콜(2026-09-12): 한 번에 길게 받으면 근피로로 뒤로 갈수록 신호가
빠져서(s3 사례: 2587 -> 672 -> 356 -> 371) 라벨이 오염된다. 그래서
"짧게 여러 번 + 사이에 휴식"으로 나눠 받고, 매 회차 평균을 즉시 찍어서
힘이 빠지고 있으면 수집 중에 바로 알 수 있게 한다.

기록 정책(2026-09-22 저녁, 최종): 판정으로 회차를 걸러내지 않는다.
사람의 근전도는 원래 일정하지 않고 사람마다 파형이 다르다. "정상 범위"를 정해 놓고
거기서 벗어난 회차를 버리면 그건 측정이 아니라 선별이다. 그래서 데이터가 들어오기만 하면
회차를 그대로 기록하고, 대신 회차마다 통계를 summary/{subject}_{gesture}_reps.csv 에 남겨
신호가 어떻게 흔들렸는지가 결과의 일부가 되게 한다.

  - 본 CSV: 0(ADC 바닥 = 끊김)·4095(포화) 만 뺀 파형. rest 는 4095 만 뺀다
  - raw/{subject}_{gesture}_raw.csv: 필터 전 전체 샘플 (무엇을 뺐는지 항상 확인 가능)
  - summary/{subject}_{gesture}_reps.csv: 회차별 유효%, 중앙값, 사분위(25/75), 앞/뒤 0.5초 중앙값, 표준편차
    (analysis/compare_thresholds.py 가 *_rest*.csv 를 긁으므로 최상위에 두면 안 됨)
  - {subject}_mvc.csv: grip 류 시작 전 "최대한 세게" 기준 쥐기 1회 (%MVC 정규화용). 판정 없음
  - --check 를 붙이면 예전 판정 규칙(자기 MVC 대비 비율·파형 모양)으로 걸러내고 재시도한다.
    기본은 꺼짐.

사용 예:
  # 휴식 데이터 (한 번에 길게 받아도 됨 - 힘을 안 주니 피로가 없음)
  python3 collect.py --port /dev/cu.usbserial-110 --subject s5 --gesture rest --seconds 10 --reps 1

  # 편한 쥐기 (물건 집듯이, 3초씩 6회) — 시작 시 rest 5초 + 최대 수축 3초(MVC) 도 같이 기록됨
  python3 collect.py --port /dev/cu.usbserial-110 --subject s5 --gesture light --seconds 3 --reps 6 --rest-between 5

  # 최대 쥐기 (3초씩 6회) — MVC 는 light 때 받았으니 생략
  python3 collect.py --port /dev/cu.usbserial-110 --subject s5 --gesture grip --seconds 3 --reps 6 --rest-between 5 --mvc-seconds 0

  → analysis/compare_thresholds.py 가 rest / light / grip 을 읽어 B(편한 쥐기 보정) vs C(최대 수축 보정) 를 비교한다

저장: "{subject}_{gesture}.csv" (있으면 이어쓰기)
"""

import argparse
import csv
import os
import statistics
import threading
import time

import serial


class SerialPump:
    """포트를 연 순간부터 쉬지 않고 읽어서 버퍼가 밀리지 않게 하는 백그라운드 리더.

    (2026-09-12) 기존에는 sleep 후 reset_input_buffer()로 비우려 했는데, macOS
    USB-시리얼 드라이버 버퍼까지는 비워지지 않아서 측정 시작 직후 7초치 과거
    데이터가 먼저 읽혔다. 그래서 serial_reader.py처럼 항상 읽어 두는 구조로 바꿈.
    """

    def __init__(self, ser):
        self.ser = ser
        self._buf = []
        self._capturing = False
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if "," not in line:
                continue
            t, raw = line.split(",", 1)
            try:
                v = int(raw)
            except ValueError:
                continue
            # ESP32 ADC는 12비트(0~4095). 깨진 줄에서 나온 범위 밖 값은 버린다.
            if not (0 <= v <= 4095):
                continue
            with self._lock:
                if self._capturing:
                    self._buf.append(([t, raw], v))

    def capture(self, seconds):
        """지금부터 seconds 동안 들어오는 '실시간' 데이터만 모은다."""
        with self._lock:
            self._buf = []
            self._capturing = True
        time.sleep(seconds)
        with self._lock:
            self._capturing = False
            out = list(self._buf)
        return [r for r, _ in out], [v for _, v in out]

    def stop(self):
        self._running = False
        self._thread.join(timeout=0.5)


def parse_args():
    p = argparse.ArgumentParser(description="EMG raw 신호 수집 (반복 + 피로 감시)")
    p.add_argument("--port", default="/dev/cu.usbserial-110")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--subject", required=True, help="예: s1, s5 ...")
    p.add_argument("--gesture", required=True, help="예: rest, grip, light50 ...")
    p.add_argument("--seconds", type=float, default=3.0, help="1회당 수집 시간(초)")
    p.add_argument("--reps", type=int, default=1, help="반복 횟수")
    p.add_argument("--rest-between", type=float, default=5.0, help="회차 사이 휴식(초)")
    p.add_argument("--countdown", type=int, default=3, help="각 회차 시작 전 준비시간(초)")
    p.add_argument(
        "--lead-in",
        type=float,
        default=1.0,
        help="'쥐세요' 신호 후 실제 측정을 시작하기까지 기다리는 시간(초). "
        "힘을 주는 상승 구간이 데이터에 섞이지 않게 하기 위함. rest 수집 땐 0으로.",
    )
    # --- 자동 거부 ---
    p.add_argument("--rest-seconds", type=float, default=5.0,
                   help="grip 류 수집 시작 전 rest 기준선 측정 시간(초). 0이면 기준선 없이 --min-mean 만 사용")
    p.add_argument("--min-mean", type=float, default=0.0,
                   help="grip 회차 절대 하한(ADC counts). 기본 0 = 안 씀. 사람마다 진폭이 달라 절대값 판정은 하지 않는다")
    p.add_argument("--min-ratio", type=float, default=0.3,
                   help="grip 회차 중앙값이 자기 MVC(기준 쥐기) 중앙값의 몇 배 이상이어야 합격인지")
    p.add_argument("--mvc-seconds", type=float, default=3.0,
                   help="grip 류 시작 전 '최대한 세게' 기준 쥐기 시간(초). 0이면 생략(--min-mean 필요)")
    p.add_argument("--rest-max", type=float, default=300.0,
                   help="rest 수집 합격 기준: 중앙값이 이 값 미만, 이 값 초과 샘플이 5%% 미만")
    p.add_argument("--sep-std", type=float, default=5.0,
                   help="grip 평균이 rest 중앙값보다 rest 표준편차의 몇 배 이상 커야 합격인지")
    p.add_argument("--max-fail", type=int, default=0,
                   help="연속 불합격 허용 횟수. 0이면 합격이 --reps 만큼 채워질 때까지 무제한 (Ctrl+C 로 중단, 합격분은 저장됨)")
    p.add_argument("--min-valid", type=float, default=20.0,
                   help="grip 회차에서 0·4095 를 뺀 유효 샘플이 전체의 몇 %% 이상이어야 판정하는지")
    p.add_argument("--check", action="store_true",
                   help="회차 판정 켜기(자기 MVC 대비 비율·파형 모양으로 걸러내고 재시도). 기본은 판정 없이 전부 기록")
    p.add_argument("--no-filter", action="store_true", help="실시간 필터 끄기 (0·4095 도 본 CSV 에 기록)")
    return p.parse_args()


def split_valid(rows, values, gesture):
    """(유효 rows, 유효 values, 전체 대비 유효 %). grip 류는 0·4095 제외, rest 는 4095 만 제외."""
    lo = -1 if gesture == "rest" else 0
    keep = [(r, v) for r, v in zip(rows, values) if lo < v < 4095]
    pct = len(keep) / len(values) * 100 if values else 0.0
    return [r for r, _ in keep], [v for _, v in keep], pct


def _edge_medians(values, hz_guess=500, edge_sec=0.5):
    """회차 앞/뒤 edge_sec 구간 중앙값. 샘플 수가 적으면 반씩 나눔."""
    n = max(1, min(int(hz_guess * edge_sec), len(values) // 2))
    return statistics.median(values[:n]), statistics.median(values[-n:])


def check_mvc(valid, valid_pct):
    """기준 쥐기(MVC) 가 쓸 만한지. 높이는 안 본다 — 살아 있고 3초 유지됐는지만."""
    if not valid:
        return False, "유효 파형 없음 (전부 0 또는 4095)"
    if valid_pct < 50:
        return False, f"유효 파형 {valid_pct:.0f}% < 50% (끊김이 많음)"
    if statistics.pstdev(valid) < 20:
        return False, f"파형 표준편차 {statistics.pstdev(valid):.0f} < 20 (고정 전압)"
    head, tail = _edge_medians(valid)
    if head > 0 and tail < head * 0.5:
        return False, f"3초 안에 신호 빠짐: 앞 {head:.0f} → 뒤 {tail:.0f}"
    return True, "ok"


def check_rep(valid, valid_pct, args, rest_med, rest_std, mvc_med, accepted_meds):
    """(합격 여부, 사유). valid 는 이미 0·4095 를 뺀 파형. 절대 높이 기준은 쓰지 않는다."""
    if not valid:
        return False, "유효 파형 없음 (전부 0 또는 4095)"
    med = statistics.median(valid)

    if args.gesture == "rest":
        if med >= args.rest_max:
            return False, f"rest 중앙값 {med:.0f} >= {args.rest_max:.0f} (수축 섞임)"
        high = sum(1 for v in valid if v > args.rest_max) / len(valid) * 100
        if high >= 5:
            return False, f"rest 중 {args.rest_max:.0f} 초과 샘플 {high:.0f}% >= 5% (지속 수축 섞임)"
        return True, "ok"

    if valid_pct < args.min_valid:
        return False, f"유효 파형 {valid_pct:.0f}% < {args.min_valid:.0f}% (거의 끊김)"
    sd = statistics.pstdev(valid) if len(valid) > 1 else 0.0
    if sd < 20:
        return False, f"파형 표준편차 {sd:.0f} < 20 (고정 전압, 살아있는 EMG 아님)"
    if args.min_mean > 0 and med < args.min_mean:
        return False, f"중앙값 {med:.0f} < 절대 하한 {args.min_mean:.0f}"
    if mvc_med:
        need = mvc_med * args.min_ratio
        if med < need:
            return False, f"중앙값 {med:.0f} < 자기 MVC {mvc_med:.0f} x {args.min_ratio:.0%} = {need:.0f} (자기 기준 대비 안 쥠)"
    if rest_med is not None and rest_std > 0:
        need = rest_med + rest_std * args.sep_std
        if med < need:
            return False, f"중앙값 {med:.0f} < rest 기준선 {rest_med:.0f} + {args.sep_std:.0f}σ({need:.0f})"
    head, tail = _edge_medians(valid)
    if head > 0 and tail < head * 0.5:
        return False, f"회차 안에서 신호 빠짐: 앞 {head:.0f} → 뒤 {tail:.0f}"
    if accepted_meds:
        ref = statistics.median(accepted_meds)
        if med < ref * 0.4:
            return False, f"중앙값 {med:.0f} < 합격 회차 중앙값 {ref:.0f} x 0.4 (붕괴)"
    return True, "ok"


def main():
    args = parse_args()
    fname = f"{args.subject}_{args.gesture}.csv"
    rej_dir = "rejected"
    rej_fname = os.path.join(rej_dir, f"{args.subject}_{args.gesture}_rejected.csv")
    raw_dir = "raw"
    raw_fname = os.path.join(raw_dir, f"{args.subject}_{args.gesture}_raw.csv")

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # ESP32 리셋 대기
    pump = SerialPump(ser)  # 이 시점부터 계속 읽어서 버퍼가 밀리지 않게 함

    all_rows = []      # 본 CSV (필터 통과분)
    raw_rows = []      # 합격 회차의 전체 샘플 (필터 전)
    rej_rows = []
    rep_means = []     # 기록된 회차 중앙값 (이름은 기존 피로 분석 코드와 호환용)
    rep_summary = []   # 회차별 통계 → {subject}_{gesture}_reps.csv

    # --- rest 기준선 (grip 류만) ---
    rest_med = rest_std = None
    if args.gesture != "rest" and args.rest_seconds > 0:
        print(f"\n[기준선] 힘 완전히 빼고 가만히... ({args.rest_seconds:.0f}초)")
        for i in range(args.countdown, 0, -1):
            print(f"   {i}")
            time.sleep(1)
        _, rv = pump.capture(args.rest_seconds)
        rv = [v for v in rv if v < 4095]  # 포화 스파이크만 제외
        if rv:
            rest_med = statistics.median(rv)
            # 표준편차는 스파이크 몇 개에 수천까지 뛰어 기준선이 ADC 상한을 넘는 일이 있었다
            # (18:30 수집: 중앙값 0, σ 1650 → 기준 8248). 스파이크에 안 흔들리는 MAD 로 잡는다.
            rest_std = 1.4826 * statistics.median(abs(v - rest_med) for v in rv)
            high = sum(1 for v in rv if v > args.rest_max) / len(rv) * 100
            print(f"   rest 중앙값={rest_med:.0f}, 산포(MAD)={rest_std:.0f}, 0값={sum(1 for v in rv if v == 0)/len(rv)*100:.0f}%, "
                  f"{args.rest_max:.0f} 초과={high:.0f}%")
            if high >= 5:
                print(f"   (기준선 측정 중 {args.rest_max:.0f} 초과가 {high:.0f}% — 기록에 남김)")
                rest_std = 0.0
        else:
            print("   ⚠ rest 데이터 없음. 연결 확인 필요")

    # --- 기준 쥐기 MVC (grip 류만). 될 때까지 반복 ---
    mvc_med = None
    if args.gesture != "rest" and args.mvc_seconds > 0:
        mvc_try = 0
        try:
            while True:
                mvc_try += 1
                print(f"\n[기준 쥐기] (시도 {mvc_try}) 준비... 신호가 오면 최대한 세게 쥐고 {args.mvc_seconds:.0f}초 유지")
                for i in range(args.countdown, 0, -1):
                    print(f"   {i}")
                    time.sleep(1)
                print("   ▶ 최대한 세게!")
                if args.lead_in > 0:
                    time.sleep(args.lead_in)
                print(f"   ● 측정 중... ({args.mvc_seconds:.0f}초)")
                rows, values = pump.capture(args.mvc_seconds)
                v_rows, v_vals, v_pct = split_valid(rows, values, args.gesture)
                ok, reason = check_mvc(v_vals, v_pct) if args.check else (True, "기록")
                # 합격/불합격 상관없이 기준 쥐기 시도는 전부 raw/ 에 남긴다 (파형 분석용. 19:00 시도 7회가 안 남아 있었음)
                os.makedirs(raw_dir, exist_ok=True)
                mvc_raw = os.path.join(raw_dir, f"{args.subject}_mvc_attempts_raw.csv")
                with open(mvc_raw, "a", newline="") as f:
                    w = csv.writer(f)
                    if f.tell() == 0:
                        w.writerow(["t_ms", "raw", "subject", "gesture", "attempt", "result"])
                    w.writerows([r + [args.subject, "mvc", mvc_try, "ok" if ok else reason] for r in rows])
                if ok:
                    mvc_med = statistics.median(v_vals)
                    line = f"   ✔ 기준 기록  유효 {v_pct:.0f}%, 중앙값={mvc_med:.0f}"
                    if args.check:
                        line += f" → 회차 합격선 {mvc_med*args.min_ratio:.0f}"
                    print(line)
                    mvc_fname = f"{args.subject}_mvc.csv"
                    with open(mvc_fname, "a", newline="") as f:
                        w = csv.writer(f)
                        if f.tell() == 0:
                            w.writerow(["t_ms", "raw", "subject", "gesture"])
                        w.writerows([r + [args.subject, "mvc"] for r in v_rows])
                    print(f"      → {mvc_fname} 저장")
                    break
                print(f"   ✘ 다시  유효 {v_pct:.0f}%  사유: {reason}")
                print("      전극 눌러 붙이고, 손목 곧게 편 채로 다시 쥐세요.")
                if args.rest_between > 0:
                    time.sleep(args.rest_between)
        except KeyboardInterrupt:
            print(f"\n   ⏹ Ctrl+C — 기준 쥐기 {mvc_try}회 시도 파형은 {mvc_raw if mvc_try else '(없음)'} 에 있음. 종료합니다.")
            pump.stop(); ser.close(); return

    print(f"\n=== {args.subject} / {args.gesture} : {args.seconds:.0f}초 x {args.reps}회"
          f"{' (판정 켜짐)' if args.check else ' (판정 없음, 전부 기록)'} ===")
    accepted = 0
    attempt = 0
    consecutive_fail = 0
    aborted = False
    try:
        while accepted < args.reps:
            attempt += 1
            print(f"\n[{accepted + 1}/{args.reps}] (시도 {attempt}) 준비...")
            for i in range(args.countdown, 0, -1):
                print(f"   {i}")
                time.sleep(1)
            if args.lead_in > 0:
                # 먼저 쥐게 하고, 힘이 다 올라온 뒤에 측정을 시작한다.
                print(f"   ▶ 지금 쥐세요!")
                time.sleep(args.lead_in)
                print(f"   ● 측정 중... ({args.seconds:.0f}초) — 그대로 유지")
            else:
                print(f"   ▶ 시작! ({args.seconds:.0f}초)")

            rows, values = pump.capture(args.seconds)
            if args.no_filter:
                v_rows, v_vals, v_pct = rows, values, 100.0
            else:
                v_rows, v_vals, v_pct = split_valid(rows, values, args.gesture)

            if args.check:
                ok, reason = check_rep(v_vals, v_pct, args, rest_med, rest_std, mvc_med, rep_means)
            else:
                ok, reason = (bool(v_vals), "기록" if v_vals else "수신 데이터 없음")

            med = statistics.median(v_vals) if v_vals else 0.0
            if v_vals:
                q = statistics.quantiles(v_vals, n=4) if len(v_vals) >= 4 else [med, med, med]
                head, tail = _edge_medians(v_vals)
                sd = statistics.pstdev(v_vals) if len(v_vals) > 1 else 0.0
            else:
                q, head, tail, sd = [0, 0, 0], 0, 0, 0.0
            info = (f"{len(values)}샘플 중 유효 {v_pct:.0f}%, 중앙값={med:.0f} "
                    f"(25~75% {q[0]:.0f}~{q[2]:.0f}, 앞 {head:.0f}→뒤 {tail:.0f})")
            if ok:
                accepted += 1
                consecutive_fail = 0
                all_rows.extend([r + [args.subject, args.gesture] for r in v_rows])
                raw_rows.extend([r + [args.subject, args.gesture] for r in rows])
                rep_means.append(med)
                rep_summary.append([args.subject, args.gesture, accepted, attempt, len(values), round(v_pct, 1),
                                    round(med), round(q[0]), round(q[2]), round(head), round(tail), round(sd, 1),
                                    round(mvc_med) if mvc_med else "", round(rest_med) if rest_med is not None else ""])
                drop = ""
                if len(rep_means) > 1:
                    ratio = med / rep_means[0] if rep_means[0] else 0
                    drop = f"  (1회차 대비 {ratio*100:.0f}%)"
                    if ratio < 0.7:
                        drop += "  ⚠️ 힘이 빠지고 있음 — 더 쉬었다 하세요"
                print(f"   ✔ {'합격' if args.check else '기록'}  {info}{drop}")
                if v_pct < 50:
                    print(f"      ⚠️ 끊김 {100-v_pct:.0f}% — 케이블/스냅 고정 확인")
            else:
                consecutive_fail += 1
                rej_rows.extend([r + [args.subject, args.gesture, attempt, reason] for r in rows])
                lim = f"/{args.max_fail}" if args.max_fail > 0 else ""
                print(f"   ✘ 불합격 (연속 {consecutive_fail}{lim})  {info}")
                print(f"      사유: {reason}")
                if args.max_fail > 0 and consecutive_fail >= args.max_fail:
                    print(f"\n   ⛔ 연속 {args.max_fail}회 불합격 → 쥐는 방법이 아니라 하드웨어 문제입니다.")
                    print("      전극·REF·케이블 확인 후 sensor_check.py 로 파형을 보고 다시 시작하세요.")
                    aborted = True
                    break
                if consecutive_fail % 3 == 0:
                    print("      ⚠️ 연속 3회 — 전극·REF·케이블을 한 번 눌러 고정하고 계속하세요 (그만두려면 Ctrl+C, 합격분은 저장됨)")
                else:
                    print("      전극 눌러 붙이고 다시 쥐세요.")

            if accepted < args.reps and args.rest_between > 0:
                print(f"   ... {args.rest_between:.0f}초 휴식 (힘 완전히 빼세요)")
                time.sleep(args.rest_between)
    except KeyboardInterrupt:
        print(f"\n   ⏹ Ctrl+C — 지금까지 합격 {accepted}회만 저장하고 종료합니다.")
        aborted = True

    pump.stop()
    ser.close()

    if all_rows:
        file_exists = os.path.exists(fname)
        with open(fname, "a", newline="") as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow(["t_ms", "raw", "subject", "gesture"])
            w.writerows(all_rows)

    if rep_summary:
        os.makedirs("summary", exist_ok=True)
        sum_fname = os.path.join("summary", f"{args.subject}_{args.gesture}_reps.csv")
        file_exists = os.path.exists(sum_fname)
        with open(sum_fname, "a", newline="") as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow(["subject", "gesture", "rep", "attempt", "n_samples", "valid_pct", "median", "q25", "q75",
                            "head_med", "tail_med", "std", "mvc_median", "rest_median"])
            w.writerows(rep_summary)

    if raw_rows and not args.no_filter:
        os.makedirs(raw_dir, exist_ok=True)
        file_exists = os.path.exists(raw_fname)
        with open(raw_fname, "a", newline="") as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow(["t_ms", "raw", "subject", "gesture"])
            w.writerows(raw_rows)

    if rej_rows:
        os.makedirs(rej_dir, exist_ok=True)
        file_exists = os.path.exists(rej_fname)
        with open(rej_fname, "a", newline="") as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow(["t_ms", "raw", "subject", "gesture", "attempt", "reason"])
            w.writerows(rej_rows)

    status = "중단" if aborted else "완료"
    print(f"\n=== {status}: {'합격' if args.check else '기록'} {accepted}/{args.reps}회, 시도 {attempt}회, {len(all_rows)}샘플 -> {fname} ===")
    if rep_summary:
        print(f"회차별 통계 -> {sum_fname}")
    if raw_rows and not args.no_filter:
        dropped = len(raw_rows) - len(all_rows)
        print(f"필터로 뺀 샘플 {dropped}개 ({dropped/len(raw_rows)*100:.0f}%), 필터 전 전체 -> {raw_fname}")
    if rej_rows:
        print(f"불합격 {attempt - accepted}회 -> {rej_fname}")
    if len(rep_means) > 2:
        print(f"회차별 중앙값: {[f'{m:.0f}' for m in rep_means]}")

        # 회차 간 힘이 들쑥날쑥한 것 자체는 문제가 아니다(실사용에서도 매번 다르다).
        # 진짜 문제는 (1) 피로로 회차가 갈수록 단조롭게 무너지는 것,
        #            (2) 특정 회차가 다른 회차 대비 붕괴해서 라벨이 거짓이 되는 것.
        n = len(rep_means)
        xs = list(range(n))
        mx, my = sum(xs) / n, sum(rep_means) / n
        cov = sum((x - mx) * (y - my) for x, y in zip(xs, rep_means))
        vx = sum((x - mx) ** 2 for x in xs) ** 0.5
        vy = sum((y - my) ** 2 for y in rep_means) ** 0.5
        corr = cov / (vx * vy) if vx and vy else 0.0

        med = statistics.median(rep_means)
        worst = min(rep_means)

        # 방향(상관)만 보면 20% 정도 완만히 내려가는 정상 케이스도 걸린다.
        # s3의 붕괴는 4092 -> 42 (99% 감소)였으므로, 크기(마지막/처음)도 같이 본다.
        decline = rep_means[-1] / rep_means[0] if rep_means[0] else 1.0
        print(f"피로 추세(회차-평균 상관): {corr:+.2f}, 마지막/처음 = {decline:.2f}", end="")
        if corr <= -0.7 and decline < 0.6:
            print("  (회차가 갈수록 내려감 — 피로 또는 접촉 변화. 기록에 남김)")
        elif corr <= -0.7:
            print("  (완만한 하강)")
        else:
            print("  (뚜렷한 추세 없음)")

        print(f"최약 회차 / 중앙값: {worst / med:.2f}", end="")
        if worst < med * 0.4:
            print("  (회차 간 편차 큼 — reps.csv 에 그대로 남김)")
        else:
            print()
    print()


if __name__ == "__main__":
    main()
