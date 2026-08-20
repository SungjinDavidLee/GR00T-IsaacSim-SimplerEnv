#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rep3.py -- gridlogs 의 RESULT 줄을 전부 모아 '성공 판과 실패 판이 무엇이 다른가' 를 낸다.

  python3 rep3.py                      # gridlogs 전체
  python3 rep3.py 'fin*' 'ver*'        # 태그 패턴 지정
  python3 rep3.py --tag-table          # 태그별 요약만

 fit=1 인데 ok=0 인 판(조준은 됐는데 못 든 판)을 따로 뽑아준다. 이게 지금의 핵심 격차다.
"""
import glob, os, re, sys
import statistics as st

LOGDIR = os.environ.get("LOGDIR", "/home/data/groot/work/gridlogs")
NUM = ("dx", "dy", "dz", "lift", "close_step", "nstep", "nclose",
       "clampx", "clampy", "clampz", "detx", "dety", "detz", "last_dy")


def load(pats):
    rows = []
    for p in pats:
        for f in sorted(glob.glob(os.path.join(LOGDIR, p + "_dx*_dy*_r*.log"))):
            t = open(f, errors="ignore").read()
            m = re.findall(r"^RESULT (.*)$", t, re.M)
            if not m:
                continue
            d = {"log": os.path.basename(f)}
            for tok in m[-1].split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    d[k] = v
            b = os.path.basename(f)
            try:
                d["can_dx"] = float(b.split("_dx")[1].split("_dy")[0])
                d["can_dy"] = float(b.split("_dy")[1].split("_r")[0])
            except Exception:
                d["can_dx"] = d["can_dy"] = float("nan")
            for k in NUM:
                try:
                    d[k] = float(d.get(k, "nan"))
                except ValueError:
                    d[k] = float("nan")
            for k in ("ok", "fit", "abort"):
                try:
                    d[k] = int(float(d.get(k, 0)))
                except ValueError:
                    d[k] = 0
            rows.append(d)
    return rows


def mm(v):
    return v * 1000 if v == v and abs(v) < 5 else float("nan")


def fmt(v, w=8, p=1):
    return ("%*.*f" % (w, p, v)) if v == v else "%*s" % (w, "-")


def tag_table(rows):
    tags = {}
    for r in rows:
        tags.setdefault(r.get("tag", "?"), []).append(r)
    print("\n%-14s %4s %5s %5s %9s %9s %9s %7s %6s %6s" % (
        "tag", "n", "fit", "ok", "dx[mm]", "dy[mm]", "dz[mm]",
        "close", "ncl", "abort"))
    out = []
    for t, v in sorted(tags.items()):
        def col(k, f=mm):
            a = [f(x[k]) for x in v if x[k] == x[k]]
            return st.mean(a) if a else float("nan")
        ok = sum(x["ok"] for x in v)
        print("%-14s %4d %5d %5d %s %s %s %s %s %s" % (
            t, len(v), sum(x["fit"] for x in v), ok,
            fmt(col("dx")), fmt(col("dy")), fmt(col("dz")),
            fmt(col("close_step", float), 7), fmt(col("nclose", float), 6),
            fmt(col("abort", float), 6, 2)))
        out.append((ok / max(len(v), 1), ok, len(v), t))
    out.sort(reverse=True)
    print("\n성공률 순위:")
    for rate, ok, n, t in out[:6]:
        print("  %-14s %d/%d  (%.0f%%)" % (t, ok, n, rate * 100))
    return out


def compare(rows):
    A = [r for r in rows if r["ok"] == 1]
    B = [r for r in rows if r["ok"] == 0]
    print("\n=========== 성공 %d판 vs 실패 %d판 ===========" % (len(A), len(B)))
    if not A:
        print("성공 판이 없어 비교 불가")
        return
    print("%-12s %12s %12s %10s" % ("항목", "성공 평균", "실패 평균", "차이"))
    for k, f, lab in (("dx", mm, "dx[mm]"), ("dy", mm, "dy[mm]"),
                      ("dz", mm, "dz[mm]"), ("close_step", float, "close_step"),
                      ("nclose", float, "nclose"), ("nstep", float, "nstep"),
                      ("abort", float, "abort"), ("clampx", float, "clampx")):
        a = [f(r[k]) for r in A if r[k] == r[k]]
        b = [f(r[k]) for r in B if r[k] == r[k]]
        if not a or not b:
            continue
        ma, mb = st.mean(a), st.mean(b)
        print("%-12s %12.2f %12.2f %10.2f" % (lab, ma, mb, ma - mb))
    print("\n성공 판 원값:")
    for r in A:
        print("  %-13s can(%+.2f,%+.2f) dx%7.1f dy%7.1f dz%7.1f close%3.0f ncl%2.0f lift%.3f"
              % (r.get("tag", "?"), r["can_dx"], r["can_dy"], mm(r["dx"]),
                 mm(r["dy"]), mm(r["dz"]), r["close_step"], r["nclose"], r["lift"]))

    gap = [r for r in rows if r["fit"] == 1 and r["ok"] == 0]
    print("\n=== 조준 성공했는데 못 든 판 (fit=1, ok=0) : %d판 ===" % len(gap))
    print("여기가 지금의 핵심 격차다.")
    for r in gap:
        print("  %-13s can(%+.2f,%+.2f) dx%7.1f dy%7.1f dz%7.1f close%3.0f ncl%2.0f abort%d nstep%3.0f"
              % (r.get("tag", "?"), r["can_dx"], r["can_dy"], mm(r["dx"]),
                 mm(r["dy"]), mm(r["dz"]), r["close_step"], r["nclose"],
                 r["abort"], r["nstep"]))
    if gap:
        okz = [mm(r["dz"]) for r in A if r["dz"] == r["dz"]]
        gz = [mm(r["dz"]) for r in gap if r["dz"] == r["dz"]]
        if okz and gz:
            print("\n  dz 평균  성공 %+.1f mm   /   조준성공-못듦 %+.1f mm" % (
                st.mean(okz), st.mean(gz)))
        ab = sum(r["abort"] for r in gap)
        print("  이 중 abort(캔 전도) %d판 / %d판" % (ab, len(gap)))
        print("  -> abort 가 대부분이면 닫는 순간 손가락이 캔을 친다는 뜻이다.")
        print("     abort 가 적으면 잡았다가 놓치거나 들어올리지 못한 것이다.")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    pats = args if args else ["*"]
    rows = load(pats)
    if not rows:
        sys.exit("RESULT 를 가진 로그가 없습니다: %s" % LOGDIR)
    print("로그 %d판  (%s)" % (len(rows), LOGDIR))
    tag_table(rows)
    if "--tag-table" not in sys.argv:
        compare(rows)


if __name__ == "__main__":
    main()
