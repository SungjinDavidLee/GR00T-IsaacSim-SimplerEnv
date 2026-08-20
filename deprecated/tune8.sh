#!/usr/bin/env bash
# tune8.sh -- dx 를 -28 mm 에서 -25 mm 안쪽으로 밀어넣는다. 남은 레버 2종.
#
#   bash tune8.sh sweep        3조건 x 6판 (K=8, dy=0). 약 20분
#   bash tune8.sh final <tag>  이긴 조건으로 dy = -0.05/0/+0.05 x 5판
#   bash tune8.sh wide  <tag>  격자 ±60 mm x 3판
#
# ── K=8 누적 32판에서 확정된 문턱 ────────────────────────────────────
#     성공 7판  dx 평균 **-25.9 mm**   (원값 -23.5 -24.8 -26.6 -27.0 -27.5 -28.4 -29.6)
#     실패     dx 평균 **-30.3 mm**
#   → |dx| 가 28 mm 를 넘으면 손가락이 캔 옆면을 쳐서 넘어뜨린다 (실패의 70 % abort=1).
#
# ── 왜 CAMSHIFT x 로는 더 못 가는가 ──────────────────────────────────
#     x -0.0168 -> dx -28.9      x -0.0180 -> dx -28.4     계수 약 -0.42 mm/mm
#     필요한 +4.4 mm 를 얻으려면 x = -0.0285 인데
#     **x = -0.0205 부터 모델이 아예 닫지 않는다** (절벽, 12판 전부 close_step=-1).
#   → x 축은 막혔다. 다른 축으로 같은 효과를 내야 한다.
#
# ── 이번에 시험하는 것 ────────────────────────────────────────────────
#   A. CAMSHIFT z  — 아직 한 번도 안 건드린 축.
#      카메라를 손가락 쪽(+z)으로 내리면 같은 횡방향 오차가 화면에서 더 크게 보인다.
#      정책이 그만큼 더 다가가서 멈출 것으로 본다. x 절벽과 무관한 경로다.
#   B. ROT6      — 지금 회전 출력을 통째로 버리고 있다 (chunk[:,3:6]=0, target_q=q0).
#      16번 §2-3 에서 "복원 가능"으로 남겨둔 항목이며, ANCHOR=detect + K=8 에서는
#      한 번도 시험하지 않았다. 접근 자세가 맞으면 dx 문턱 자체가 완화된다.
set -u
MODE="${1:-sweep}"
ARG="${2:-}"
W=/home/data/groot/work
export LOGDIR=$W/gridlogs
cd /home/data/groot/MIGRATE/ENV
source /home/data/groot/venv-isaacsim/bin/activate
unset CUDA_VISIBLE_DEVICES MUJOCO_GL PYOPENGL_PLATFORM
export NOWIN=1 OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/data/groot/MIGRATE/ENV
export RUNNER=$W/run_groot31.py
export VIDROOT=$W/vid VIDFPS=20 VIDEVERY=4
mkdir -p "$VIDROOT"

export WCROP=1 WROT=0 WFLIP=0 FCROP=1
export SETTLE=8 RESEED=0 IDLE=1
export ASCALE=0.05 NSUB=8 ZBIAS=0 XBIAS=-0.19 YBIAS=0.105
export GLATCH=0 ARMONLY=1 LIFTTH=0.08 GDEB=3 GHOLD=24
export ZSCALE=1.0 GOFF=0.018 YMAX=0.106 FLOORB=0.0
export INSTR="pick up the tomato sauce and place it in the basket"
export ANCHOR=detect CANRAD=0.0329 CANRK=0.72 CANZOFF=-0.0077
export CAMKEY=wrist CAMMODE=wristdup
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1 NOREOPEN=0
export FITDX=0.045 FITDY=0.0063 FITDZ=0.025
export KSAMP=1 KSAMPOPEN=${K:-8} KLOG=0

python3 $W/front_mode.py original || exit 1

wilson () {
  python3 - "$1" "$2" <<'EOF'
import csv, math, sys
r = list(csv.DictReader(open(sys.argv[1])))
k = sum(int(x["ok"]) for x in r); n = len(r)
if not n: sys.exit("행 없음")
z = 1.96; p = k / n; d = 1 + z * z / n
c = (p + z * z / (2 * n)) / d
h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
print("\n== %s ==  %d/%d = %.0f%%   95%% 구간 %.0f~%.0f%%" % (
    sys.argv[2], k, n, 100 * p, 100 * max(0, c - h), 100 * min(1, c + h)))
print("   K=8 누적 대조군 7/32 = 22 %  (11~39 %)")
EOF
}

setcond () {
  export CAMSHIFT="-0.0180,0.0283,0"
  unset ROT6 RSCALE RMAX 2>/dev/null || true
  case "$1" in
    t8_cz10) export CAMSHIFT="-0.0180,0.0283,0.010" ;;
    t8_cz20) export CAMSHIFT="-0.0180,0.0283,0.020" ;;
    t8_r6)   export ROT6=1 RSCALE=0.05 RMAX=0.5 ;;
    t8_base) : ;;
    *) echo "알 수 없는 조건: $1"; exit 1 ;;
  esac
}

run () {   # run <tag> <dylist> <reps>
  setcond "$1"
  echo
  echo "########## $1  CAMSHIFT=$CAMSHIFT  ROT6=${ROT6:-0}  K=$KSAMPOPEN  dy=$2 x$3"
  TAG=$1 python3 $W/pos_grid.py --dx="${DXL:-0}" --dy="$2" --reps "$3" \
      --tag "$1" --out $W/exp_$1.csv --logdir $W/gridlogs
}

case "$MODE" in
  sweep)
    for C in t8_cz10 t8_cz20 t8_r6; do ( run $C "0" 6 ); done
    python3 $W/rep3.py 't8_*'
    for C in t8_cz10 t8_cz20 t8_r6; do wilson $W/exp_$C.csv "$C"; done
    echo
    echo "볼 것: dx 평균이 -26 mm 안쪽으로 오는가 / abort 가 줄어드는가."
    echo "close_step=-1 이 나오면 그 조건은 절벽이니 버린다."
    ;;
  final)
    [ -n "$ARG" ] || { echo "조건 지정: bash tune8.sh final t8_cz10"; exit 1; }
    ( run "$ARG" "0" 0 ) 2>/dev/null || true
    setcond "$ARG"
    TAG=f_$ARG python3 $W/pos_grid.py --dx=0 --dy="-0.05,0,0.05" --reps 5 \
        --tag "f_$ARG" --out $W/exp_f_$ARG.csv --logdir $W/gridlogs
    python3 $W/rep3.py "f_$ARG"
    wilson $W/exp_f_$ARG.csv "최종 $ARG, 캔 ±50 mm"
    ;;
  wide)
    [ -n "$ARG" ] || { echo "조건 지정"; exit 1; }
    setcond "$ARG"
    export DXL="-0.06,0,0.06"
    TAG=w_$ARG python3 $W/pos_grid.py --dx="$DXL" --dy="-0.06,0,0.06" --reps 3 \
        --tag "w_$ARG" --out $W/exp_w_$ARG.csv --logdir $W/gridlogs
    python3 $W/rep3.py "w_$ARG"
    wilson $W/exp_w_$ARG.csv "격자 ±60 mm, $ARG"
    ;;
  *) echo "usage: bash tune8.sh [sweep|final <tag>|wide <tag>]"; exit 1 ;;
esac
