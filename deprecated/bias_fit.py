#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bias_fit.py -- 스윕 CSV 의 '첫 파지 잔차' 로 XBIAS / YBIAS 보정값을 계산한다.

 배경: XBIAS/YBIAS 는 잔여 계통오차 보정이 아니라 **GR00T 의 기억된 궤적 종점을
 캔 위치에 맞추는 정렬 상수**다(16번 §1-2). 따라서 관측 구성이 바뀌면
 (CAMMODE, front 위치, 체크포인트) 다시 잡아야 한다.

 ANCHOR=detect/follow 에서는 프레임이 캔을 따라가므로 잔차는 위치와 무관하게
 일정해야 한다. 일정하지 않으면 정렬 상수만으로는 못 잡는다는 뜻이므로
 그 사실도 함께 출력한다.

 사용:
   python3 bias_fit.py exp_detect.csv                 # 현재 XBIAS/YBIAS 를 env 에서 읽음
   python3 bias_fit.py exp_detect.csv -0.19 0.105     # 직접 지정
"""
import csv, os, sys
import statistics as st

path = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work/exp_detect.csv"
XB = float(sys.argv[2]) if len(sys.argv) > 2 else float(os.environ.get("XBIAS", "-0.19"))
YB = float(sys.argv[3]) if len(sys.argv) > 3 else float(os.environ.get("YBIAS", "0.105"))

rows = list(csv.DictReader(open(path)))


def f(r, k):
    try:
        return float(r[k])
    except (ValueError, TypeError, KeyError):
        return None


use = [r for r in rows if f(r, "err_dy") is not None and f(r, "err_dx") is not None]
if not use:
    sys.exit("첫 파지 기록이 있는 행이 없습니다: " + path)

print("행 %d / 전체 %d  (닫기 없는 판은 제외)" % (len(use), len(rows)))
print("\n%8s %8s %5s %10s %10s %10s %10s" % (
    "dx", "dy", "n", "dx[mm]", "dy[mm]", "det_dx[mm]", "det_dy[mm]"))
cells = {}
for r in use:
    cells.setdefault((r["dx"], r["dy"]), []).append(r)
for (dx, dy), rs in sorted(cells.items(), key=lambda kv: (float(kv[0][1]), float(kv[0][0]))):
    ex = [f(r, "err_dx") * 1000 for r in rs]
    ey = [f(r, "err_dy") * 1000 for r in rs]
    dtx = [f(r, "det_dx") * 1000 for r in rs if f(r, "det_dx") is not None]
    dty = [f(r, "det_dy") * 1000 for r in rs if f(r, "det_dy") is not None]
    print("%8s %8s %5d %10.1f %10.1f %10s %10s" % (
        dx, dy, len(rs), st.mean(ex), st.mean(ey),
        "%.1f" % st.mean(dtx) if dtx else "-",
        "%.1f" % st.mean(dty) if dty else "-"))

# ---- 정렬 상수는 캔이 원위치(dx=dy=0)인 셀에서 잡는다 ----
ctr = [r for r in use if abs(float(r["dx"])) < 1e-9 and abs(float(r["dy"])) < 1e-9]
src = ctr if ctr else use
if not ctr:
    print("\n※ dx=dy=0 셀이 없어 전체 평균으로 계산합니다.")
mx = st.mean([f(r, "err_dx") for r in src])
my = st.mean([f(r, "err_dy") for r in src])
sx = st.pstdev([f(r, "err_dx") for r in src])
sy = st.pstdev([f(r, "err_dy") for r in src])

print("\n기준 셀 잔차  dx %+.4f ±%.4f   dy %+.4f ±%.4f  (n=%d)" % (mx, sx, my, sy, len(src)))
print("""
  유도 (v2 에서 부호 정정):
    lp = RbT(ep-bp) - OFF,   OFF = RbT(can-bp) - LIB_OBJ - bias
    -> err = RbT(ep-can) = lp - LIB_OBJ - bias
    -> err_new = err_old + (bias_old - bias_new)
    -> **bias_new = bias_old + err_old**   (빼는 게 아니라 더한다)""")
print("\n== 제안값 ==")
print("  XBIAS  %+.4f  ->  %+.4f" % (XB, XB + mx))
print("  YBIAS  %+.4f  ->  %+.4f" % (YB, YB + my))
print("\n  XBIAS=%.4f YBIAS=%.4f bash exp_det.sh detect 3" % (XB + mx, YB + my))

# ---- 클램프 포화 점검: lp 를 역산해 상자 경계에 붙어 있는지 본다 ----
LIB_OBJ = (0.05, -0.10, 0.035)
CLO = (-0.176, -0.294)
CHI = (0.146, float(os.environ.get("YMAX", "0.106")))
lpx, lpy = mx + LIB_OBJ[0] + XB, my + LIB_OBJ[1] + YB
print("\n== 클램프 포화 점검 (기준 셀에서 역산한 lp) ==")
print("  lp_x %+.4f   상자 x [%.3f, %.3f]" % (lpx, CLO[0], CHI[0]))
print("  lp_y %+.4f   상자 y [%.3f, %.3f]" % (lpy, CLO[1], CHI[1]))
for nm, v, lo, hi in (("x", lpx, CLO[0], CHI[0]), ("y", lpy, CLO[1], CHI[1])):
    if abs(v - lo) < 0.005 or abs(v - hi) < 0.005:
        print("  !! lp_%s 가 상자 경계에 붙어 있다 -> 이 축은 정책이 아니라 클램프가 정한다." % nm)
        print("     bias 만 바꾸면 상자가 같이 움직여 물리 위치가 안 변할 수 있다.")
        print("     CLAMPPAD / CLAMPOFF 로 A/B 할 것.")

# ---- 위치 의존성 점검: 정렬 상수로 해결 가능한 문제인가 ----
allx = [f(r, "err_dx") for r in use]
ally = [f(r, "err_dy") for r in use]
rx, ry = max(allx) - min(allx), max(ally) - min(ally)
print("\n== 전 셀 잔차 변동폭 ==")
print("  dx %.1f mm   dy %.1f mm   (허용 dy ±6.3 mm)" % (rx * 1000, ry * 1000))
if ry < 0.0126:
    print("  -> 변동폭이 허용범위 안. **상수 하나로 전 위치를 맞출 수 있다.**")
else:
    print("  -> 변동폭이 허용범위보다 크다. 상수 보정만으로는 일부 위치가 남는다.")
    print("     원인 후보: (a) 캔이 움직이면 wrist 영상이 바뀌어 궤적 종점이 흔들림")
    print("               (b) 접근 중 되먹임 부족(NSUBOPEN)  (c) 작업공간 클램프")
