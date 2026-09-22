"""
EMG 원시 신호 수집.

개선된 프로토콜(2026-09-12): 한 번에 길게 받으면 근피로로 뒤로 갈수록 신호가
빠져서(s3 사례: 2587 -> 672 -> 356 -> 371) 라벨이 오염된다. 그래서
"짧게 여러 번 + 사이에 휴식"으로 나눠 받고, 매 회차 평균을 즉시 찍어서
힘이 빠지고 있으면 수집 중에 바로 알 수 있게 한다.

자동 거부(2026-09-22): s5 수집에서 6회 중 4회가 rest 수준(평균 147~449)으로
나왔는데도 그대로 저장돼 나중에 손으로 골라내야 했다. 이제는 회차가 끝나면
즉시 검사해서 불합격이면 버리고 같은 회차를 다시 받는다. 합격 회차가 --reps 만큼
모일 때까지 반복한다(기본 무제한, --max-fail N 이면 연속 N회 불합격 시 중단). Ctrl+C 로
끊으면 그때까지 합격분만 저장한다.
불합격 회차는 본 데이터에 섞지 않고 rejected/ 폴더에 따로 남긴다(원인 분석용).

실시간 필터(2026-09-22 저녁): 접촉이 들락날락하면 0(ADC 바닥 = 끊김)과 4095(포화
스파이크)가 파형 사이에 섞인다. 이 두 값은 근육 신호가 아니므로 본 CSV에는 기록하지
않는다. 걸러내기 전 전체 샘플은 raw/{subject}_{gesture}_raw.csv 에 그대로 남긴다
(무엇을 뺐는지 항상 확인 가능). 판정도 걸러낸 파형의 중앙값으로 한다.

grip 류 검사 기준(각 회차 끝날 때, 0·4095 제외한 파형 기준):
  - 유효 샘플이 전체의 --min-valid(20%) 미만이면 불합격 (파형이 거의 없음)
  - 중앙값 >= rest 중앙값 + rest 표준편차 x --sep-std, 그리고 중앙값 >= --min-mean
  - 마지막 0.5초 중앙값 >= 첫 0.5초 중앙값 x 0.5  (회차 안에서 신호가 빠짐)
  - 중앙값 >= 이미 합격한 회차 중앙값 x 0.4  (s3식 붕괴)
rest 검사 기준(4095 만 제외, 0은 실제 기준선이라 남김):
  - 중앙값 < --min-mean 의 절반  (수축이 섞여 중앙값까지 올라오면 불합격)
  - --min-mean 초과 샘플 < 5%   (1~2초짜리 지속 수축 차단. 순간 스파이크는 5% 까지 허용)

사용 예:
  # 휴식 데이터 (한 번에 길게 받아도 됨 - 힘을 안 주니 피로가 없음)
  python3 collect.py --port /dev/cu.usbserial-110 --subject s5 --gesture rest --seconds 10 --reps 1

  # 쥐기 데이터 (3초씩 6회, 사이에 5초 휴식)
  python3 collect.py --port /dev/cu.usbserial-110 --subject s5 --gesture grip --seconds 3 --reps 6 --rest-between 5

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
    p.add_argument("--min-mean", type=float, default=600.0,
                   help="grip 회차 합격에 필요한 최소 평균(ADC counts). 편한 쥐기 평균의 절반쯤으로, MyoWare 게인 바꾸면 같이 조정 "
                        "(s5 실패 회차는 최대 449, s1_v5 정상 회차 최소 753)")
    p.add_argument("--sep-std", type=float, default=5.0,
                   help="grip 평균이 rest 중앙값보다 rest 표준편차의 몇 배 이상 커야 합격인지")
    p.add_argument("--max-fail", type=int, default=0,
                   help="연속 불합격 허용 횟수. 0이면 합격이 --reps 만큼 채워질 때까지 무제한 (Ctrl+C 로 중단, 합격분은 저장됨)")
    p.add_argument("--min-valid", type=float, default=20.0,
                   help="grip 회차에서 0·4095 를 뺀 유효 샘플이 전체의 몇 %% 이상이어야 판정하는지")
    p.add_argument("--no-check", action="store_true", help="자동 거부 끄기 (예전 방식: 무조건 저장)")
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


def check_rep(valid, valid_pct, args, rest_med, rest_std, accepted_meds):
    """(합격 여부, 사유). valid 는 이미 0·4095 를 뺀 파형."""
    if not valid:
        return False, "유효 파형 없음 (전부 0 또는 4095)"
    med = statistics.median(valid)

    if args.gesture == "rest":
        if med >= args.min_mean * 0.5:
            return False, f"rest 중앙값 {med:.0f} >= {args.min_mean*0.5:.0f} (수축 섞임)"
        # 중앙값이 0이어도 1~2초짜리 수축이 섞이면 rest 가 아니다 (18:18 수집 1·2회차: 600 초과 15%).
        # 순간 스파이크는 5% 까지 봐준다.
        high = sum(1 for v in valid if v > args.min_mean) / len(valid) * 100
        if high >= 5:
            return False, f"rest 중 {args.min_mean:.0f} 초과 샘플 {high:.0f}% >= 5% (지속 수축 섞임)"
        return True, "ok"

    if valid_pct < args.min_valid:
        return False, f"유효 파형 {valid_pct:.0f}% < {args.min_valid:.0f}% (거의 끊김)"
    if med < args.min_mean:
        return False, f"중앙값 {med:.0f} < 최소 {args.min_mean:.0f} (rest 수준)"
    if rest_med is not None:
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
    rep_means = []     # 합격 회차 중앙값 (이름은 기존 피로 분석 코드와 호환용)

    # --- rest 기준선 (grip 류만) ---
    rest_med = rest_std = None
    if not args.no_check and args.gesture != "rest" and args.rest_seconds > 0:
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
            high = sum(1 for v in rv if v > args.min_mean) / len(rv) * 100
            print(f"   rest 중앙값={rest_med:.0f}, 산포(MAD)={rest_std:.0f}, 0값={sum(1 for v in rv if v == 0)/len(rv)*100:.0f}%, "
                  f"{args.min_mean:.0f} 초과={high:.0f}%")
            if rest_std == 0:
                print("   (rest 가 0에 붙어 있어 σ 기준은 무의미 → --min-mean 만 적용)")
            if high >= 5:
                print(f"   ⚠️ 기준선 측정 중 {args.min_mean:.0f} 초과가 {high:.0f}% — 힘이 들어가 있거나 접촉 불안정. σ 기준 끄고 --min-mean 만 적용")
                rest_std = 0.0
        else:
            print("   ⚠ rest 데이터 없음. 연결 확인 필요")

    print(f"\n=== {args.subject} / {args.gesture} : {args.seconds:.0f}초 x {args.reps}회"
          f"{' (자동 거부 꺼짐)' if args.no_check else ''} ===")
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

            if args.no_check:
                ok, reason = (bool(v_vals), "ok" if v_vals else "수신 데이터 없음")
            else:
                ok, reason = check_rep(v_vals, v_pct, args, rest_med, rest_std, rep_means)

            med = statistics.median(v_vals) if v_vals else 0.0
            info = f"{len(values)}샘플 중 유효 {v_pct:.0f}%, 중앙값={med:.0f}"
            if ok:
                accepted += 1
                consecutive_fail = 0
                all_rows.extend([r + [args.subject, args.gesture] for r in v_rows])
                raw_rows.extend([r + [args.subject, args.gesture] for r in rows])
                rep_means.append(med)
                drop = ""
                if len(rep_means) > 1:
                    ratio = med / rep_means[0] if rep_means[0] else 0
                    drop = f"  (1회차 대비 {ratio*100:.0f}%)"
                    if ratio < 0.7:
                        drop += "  ⚠️ 힘이 빠지고 있음 — 더 쉬었다 하세요"
                print(f"   ✔ 합격  {info}{drop}")
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
    print(f"\n=== {status}: 합격 {accepted}/{args.reps}회, 시도 {attempt}회, {len(all_rows)}샘플 -> {fname} ===")
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
            print("  ⚠️ 회차가 갈수록 무너지고 있습니다 (s3 사례와 같은 패턴) — 다시 받으세요")
        elif corr <= -0.7:
            print("  (완만한 하강이지만 크기는 허용 범위)")
        else:
            print("  (추세 없음 = 정상)")

        print(f"최약 회차 / 중앙값: {worst / med:.2f}", end="")
        if worst < med * 0.4:
            print("  ⚠️ 유독 약한 회차가 있습니다 — 그 회차는 grip이라 보기 어려움")
        else:
            print("  (모든 회차가 grip 수준 유지 = 정상)")
    print()


if __name__ == "__main__":
    main()
