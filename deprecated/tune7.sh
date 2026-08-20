#!/usr/bin/env bash
# tune7.sh -- 조준은 끝났다(fit 6/6, |dy| 1.7 mm). 파지 높이와 x 미세 조정만 남았다.
#
#   bash tune7.sh sweep        3조건 x 6판 (K=8, dy=0). 약 21분
#   bash tune7.sh final <FB> <X>   최종 검증 dy = -0.05/0/+0.05 x 5판
#   bash tune7.sh wide  <FB> <X>   격자 ±60 mm x 3판 (시연용)
#
# ── 확정된 것 ─────────────────────────────────────────────────────────
#  K=8 (KSAMPOPEN) 로 정책 샘플링 잡음을 줄인 뒤:
#    x=-0.0168 :  fit 6/6,  |dy| 평균 1.7 mm,  dx -28.9 mm,  ok 2/6
#    누적 대조군 (k8 8판 + x168 6판) = **4/14 = 29 %**
#  CAMSHIFT x 는 -0.0205 부터 **모델이 아예 닫지 않는다** (close_step=-1, 6판 전부).
#    3.7 mm 만에 절벽이다. 큰 폭 조정은 불가. 미세 조정만 가능하다.
#  실패의 3/4 은 abort=1 — 닫는 순간 손가락이 캔을 쳐서 넘어뜨린다.
#
#  영상 관찰(2026-08-19): "조금 더 앞으로 가서, 조금 더 높이 잡아야 한다."
#    앞으로 -> CAMSHIFT x 를 절벽 직전까지만 (-0.0180)
#    높이   -> FLOORB 를 양수로 (지금까지 0 / -0.020 / -0.035 만 봤다)
#             z_floor = z_ref + GRASP_OFFSET + FLOORB
set -u
MODE="${1:-sweep}"
A2="${2:-}"
A3="${3:-}"
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
export ZSCALE=1.0 GOFF=0.018 YMAX=0.106
export INSTR="pick up the tomato sauce and place it in the basket"
export ANCHOR=detect CANRAD=0.0329 CANRK=0.72 CANZOFF=-0.0077
export CAMKEY=wrist CAMMODE=wristdup
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1 NOREOPEN=0
export FITDX=0.045 FITDY=0.0063 FITDZ=0.025
export KSAMP=1 KSAMPOPEN=${K:-8} KLOG=0
CY=${CY:-0.0283}

python3 $W/front_mode.py original || exit 1

wilson () {   # wilson <csv> <label>
  python3 - "$1" "$2" <<'EOF'
import csv, math, sys
r = list(csv.DictReader(open(sys.argv[1])))
k = sum(int(x["ok"]) for x in r); n = len(r)
if not n:
    sys.exit("행 없음")
z = 1.96; p = k / n; d = 1 + z * z / n
c = (p + z * z / (2 * n)) / d
h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
print("\n== %s ==  %d/%d = %.0f%%   95%% 구간 %.0f~%.0f%%" % (
    sys.argv[2], k, n, 100 * p, 100 * max(0, c - h), 100 * min(1, c + h)))
print("   대조군: x=-0.0168, FLOORB=0  ->  4/14 = 29 %  (12~55 %)")
EOF
}

run () {   # run <tag> <camshift_x> <floorb> <dylist> <reps>
  export CAMSHIFT="$2,$CY,0" FLOORB="$3"
  echo
  echo "########## $1   CAMSHIFT=$CAMSHIFT  FLOORB=$FLOORB  K=$KSAMPOPEN  dy=$4 x$5"
  TAG=$1 python3 $W/pos_grid.py --dx="${DXL:-0}" --dy="$4" --reps "$5" \
      --tag "$1" --out $W/exp_$1.csv --logdir $W/gridlogs
}

case "$MODE" in
  sweep)
    #  높이만            x 만 (절벽 직전)      둘 다
    run h_fb15  "-0.0168" "0.015"  "0" 6
    run h_x180  "-0.0180" "0.0"    "0" 6
    run h_both  "-0.0180" "0.015"  "0" 6
    python3 $W/rep3.py 'h_*'
    for T in h_fb15 h_x180 h_both; do wilson $W/exp_$T.csv "$T"; done
    echo
    echo "주의: close_step=-1 / nclose=0 이 나오면 그 조건은 '모델이 닫지 않는' 절벽이다."
    echo "      dx 가 -25 mm 쪽으로 오면서 abort 가 줄어드는 조건을 고를 것."
    echo "다음:  bash tune7.sh final <FLOORB> <CAMSHIFT_x>"
    ;;
  final)
    run kfin "${A3:--0.0168}" "${A2:-0.015}" "-0.05,0,0.05" 5
    python3 $W/rep3.py 'kfin'
    wilson $W/exp_kfin.csv "최종 FLOORB=${A2:-0.015} x=${A3:--0.0168}, 캔 ±50 mm"
    ;;
  wide)
    export DXL="-0.06,0,0.06"
    run kwide "${A3:--0.0168}" "${A2:-0.015}" "-0.06,0,0.06" 3
    python3 $W/rep3.py 'kwide'
    wilson $W/exp_kwide.csv "격자 ±60 mm"
    ;;
  *) echo "usage: bash tune7.sh [sweep|final <FLOORB> <x>|wide <FLOORB> <x>]"; exit 1 ;;
esac
