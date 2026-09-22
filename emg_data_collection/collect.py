"""
EMG 원시 신호 수집.

개선된 프로토콜(2026-09-12): 한 번에 길게 받으면 근피로로 뒤로 갈수록 신호가
빠져서(s3 사례: 2587 -> 672 -> 356 -> 371) 라벨이 오염된다. 그래서
"짧게 여러 번 + 사이에 휴식"으로 나눠 받고, 매 회차 평균을 즉시 찍어서
힘이 빠지고 있으면 수집 중에 바로 알 수 있게 한다.

자동 거부(2026-09-22): s5 수집에서 6회 중 4회가 rest 수준(평균 147~449)으로
나왔는데도 그대로 저장돼 나중에 손으로 골라내야 했다. 이제는 회차가 끝나면
즉시 검사해서 불합격이면 버리고 같은 회차를 다시 받는다. 합격 회차가 --reps 만큼
모일 때까지 반복하고, 연속 --max-fail 회 불합격이면 하드웨어 문제로 보고 중단한다.
불합격 회차는 본 데이터에 섞지 않고 rejected/ 폴더에 따로 남긴다(원인 분석용).

grip 류 검사 기준(각 회차 끝날 때):
  - 평균 >= rest 중앙값 + rest 표준편차 x --sep-std, 그리고 평균 >= --min-mean
  - 0값 비율 < 10%  (ADC 바닥에 붙음 = 접촉 불량)
  - 4095 비율 < 2%  (게인 과다)
  - 마지막 0.5초 평균 >= 첫 0.5초 평균 x 0.5  (회차 안에서 신호가 빠짐)
  - 평균 >= 이미 합격한 회차 중앙값 x 0.4  (s3식 붕괴)
rest 검사 기준:
  - --min-mean 의 절반을 넘는 샘플이 2% 미만  (수축이 섞임)

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
    p.add_argument("--max-fail", type=int, default=3,
                   help="연속 불합격 허용 횟수. 넘으면 하드웨어 문제로 보고 중단")
    p.add_argument("--no-check", action="store_true", help="자동 거부 끄기 (예전 방식: 무조건 저장)")
    return p.parse_args()


def _edge_means(values, hz_guess=500, edge_sec=0.5):
    """회차 앞/뒤 edge_sec 구간 평균. 샘플 수가 적으면 반씩 나눔."""
    n = max(1, min(int(hz_guess * edge_sec), len(values) // 2))
    return statistics.mean(values[:n]), statistics.mean(values[-n:])


def check_rep(values, args, rest_med, rest_std, accepted_means):
    """(합격 여부, 사유 문자열). values 가 비어 있으면 불합격."""
    if not values:
        return False, "수신 데이터 없음"
    mean = statistics.mean(values)
    n = len(values)
    zero_pct = sum(1 for v in values if v == 0) / n * 100
    sat_pct = sum(1 for v in values if v >= 4095) / n * 100

    if args.gesture == "rest":
        contracted = sum(1 for v in values if v > args.min_mean * 0.5) / n * 100
        if contracted >= 2:
            return False, f"수축 섞임: {contracted:.0f}% 샘플이 {args.min_mean*0.5:.0f} 초과"
        return True, "ok"

    if zero_pct >= 10:
        return False, f"0값 {zero_pct:.0f}% (ADC 바닥, 접촉 불량)"
    if sat_pct >= 2:
        return False, f"4095 포화 {sat_pct:.0f}% (게인 과다)"
    if mean < args.min_mean:
        return False, f"평균 {mean:.0f} < 최소 {args.min_mean:.0f} (rest 수준)"
    if rest_med is not None:
        need = rest_med + rest_std * args.sep_std
        if mean < need:
            return False, f"평균 {mean:.0f} < rest 기준선 {rest_med:.0f} + {args.sep_std:.0f}σ({need:.0f})"
    head, tail = _edge_means(values)
    if head > 0 and tail < head * 0.5:
        return False, f"회차 안에서 신호 빠짐: 앞 {head:.0f} → 뒤 {tail:.0f}"
    if accepted_means:
        med = statistics.median(accepted_means)
        if mean < med * 0.4:
            return False, f"평균 {mean:.0f} < 합격 회차 중앙값 {med:.0f} x 0.4 (붕괴)"
    return True, "ok"


def main():
    args = parse_args()
    fname = f"{args.subject}_{args.gesture}.csv"
    rej_dir = "rejected"
    rej_fname = os.path.join(rej_dir, f"{args.subject}_{args.gesture}_rejected.csv")

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # ESP32 리셋 대기
    pump = SerialPump(ser)  # 이 시점부터 계속 읽어서 버퍼가 밀리지 않게 함

    all_rows = []
    rej_rows = []
    rep_means = []

    # --- rest 기준선 (grip 류만) ---
    rest_med = rest_std = None
    if not args.no_check and args.gesture != "rest" and args.rest_seconds > 0:
        print(f"\n[기준선] 힘 완전히 빼고 가만히... ({args.rest_seconds:.0f}초)")
        for i in range(args.countdown, 0, -1):
            print(f"   {i}")
            time.sleep(1)
        _, rv = pump.capture(args.rest_seconds)
        if rv:
            rest_med = statistics.median(rv)
            rest_std = statistics.pstdev(rv) if len(rv) > 1 else 0.0
            print(f"   rest 중앙값={rest_med:.0f}, 표준편차={rest_std:.0f}, 0값={sum(1 for v in rv if v == 0)/len(rv)*100:.0f}%")
            if rest_std == 0:
                print("   (rest 가 0에 붙어 있어 σ 기준은 무의미 → --min-mean 만 적용)")
        else:
            print("   ⚠ rest 데이터 없음. 연결 확인 필요")

    print(f"\n=== {args.subject} / {args.gesture} : {args.seconds:.0f}초 x {args.reps}회"
          f"{' (자동 거부 꺼짐)' if args.no_check else ''} ===")
    accepted = 0
    attempt = 0
    consecutive_fail = 0
    aborted = False
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

        if args.no_check:
            ok, reason = (bool(values), "ok" if values else "수신 데이터 없음")
        else:
            ok, reason = check_rep(values, args, rest_med, rest_std, rep_means)

        mean = statistics.mean(values) if values else 0.0
        if ok:
            accepted += 1
            consecutive_fail = 0
            all_rows.extend([r + [args.subject, args.gesture] for r in rows])
            rep_means.append(mean)
            drop = ""
            if len(rep_means) > 1:
                ratio = mean / rep_means[0] if rep_means[0] else 0
                drop = f"  (1회차 대비 {ratio*100:.0f}%)"
                if ratio < 0.7:
                    drop += "  ⚠️ 힘이 빠지고 있음 — 더 쉬었다 하세요"
            print(f"   ✔ 합격  {len(values)}샘플, 평균={mean:.0f}{drop}")
        else:
            consecutive_fail += 1
            rej_rows.extend([r + [args.subject, args.gesture, attempt, reason] for r in rows])
            print(f"   ✘ 불합격 ({consecutive_fail}/{args.max_fail})  {len(values)}샘플, 평균={mean:.0f}")
            print(f"      사유: {reason}")
            if consecutive_fail >= args.max_fail:
                print(f"\n   ⛔ 연속 {args.max_fail}회 불합격 → 쥐는 방법이 아니라 하드웨어 문제입니다.")
                print("      전극·REF·케이블 확인 후 sensor_check.py 로 분리도를 보고 다시 시작하세요.")
                aborted = True
                break
            print("      전극 눌러 붙이고 다시 쥐세요.")

        if accepted < args.reps and args.rest_between > 0:
            print(f"   ... {args.rest_between:.0f}초 휴식 (힘 완전히 빼세요)")
            time.sleep(args.rest_between)

    pump.stop()
    ser.close()

    if all_rows:
        file_exists = os.path.exists(fname)
        with open(fname, "a", newline="") as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow(["t_ms", "raw", "subject", "gesture"])
            w.writerows(all_rows)

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
    if rej_rows:
        print(f"불합격 {attempt - accepted}회 -> {rej_fname}")
    if len(rep_means) > 2:
        print(f"회차별 평균: {[f'{m:.0f}' for m in rep_means]}")

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
