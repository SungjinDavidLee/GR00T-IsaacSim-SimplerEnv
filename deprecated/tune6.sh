#!/usr/bin/env bash
# tune6.sh -- 잡음을 죽였으니 이제 조준을 다시 맞춘다.
#
#   bash tune6.sh xsweep   CAMSHIFT x 3값 x 6판 (K=8, dy=0). 약 25분
#   bash tune6.sh final <x>   이긴 x 로 dy = -0.05/0/+0.05 x 5판   ← 최종
#   bash tune6.sh wide  <x>   dx,dy ±0.06 격자 3판 (시연용)
#
# 왜 지금 다시 맞추는가:
#   KSAMPOPEN=8 로 정책 샘플링 잡음을 줄이자
#     조준 성공(fit)  ~30 %  ->  **7/8 (87.5 %)**
#     dy 산포          30 mm ->  **12.3 mm**,  |dy| 평균 3.7 mm
#     dx 산포        42~46 mm ->  **9.3 mm**
#   즉 조준은 이제 재현된다. 남은 것은 그 재현되는 지점이 캔 중심에서
#   dx −29.4 mm 로 **일정하게 어긋나 있다**는 것뿐이다.
#   성공 2판은 dx −27.0 / −28.4, 실패 6판은 평균 −30.0. 경계에 걸쳐 있다.
#   잡음이 있을 때의 CAMSHIFT 재교정은 무의미했지만, 지금은 의미가 있다.
set -u
MODE="${1:-xsweep}"
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
export ZSCALE=1.0 GOFF=0.018 FLOORB=0.0 YMAX=0.106
export INSTR="pick up the tomato sauce and place it in the basket"
export ANCHOR=detect CANRAD=0.0329 CANRK=0.72 CANZOFF=-0.0077
export CAMKEY=wrist CAMMODE=wristdup
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1 NOREOPEN=0
export FITDX=0.045 FITDY=0.0063 FITDZ=0.020
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
lo, hi = 100 * max(0, c - h), 100 * min(1, c + h)
print("\n== %s ==  %d/%d = %.0f%%   95%% 구간 %.0f~%.0f%%" % (
    sys.argv[2], k, n, 100 * p, lo, hi))
print("   K=1 기준선 4/37 = 11%% (4~25%%)   K=8 x=-0.0168 기준 2/8 = 25%% (7~59%%)")
print("   판정: %s" % ("**개선 확정**" if lo > 25 else "아직 구간이 겹친다"))
EOF
}

run () {   # run <tag> <camshift_x> <dylist> <reps>
  export CAMSHIFT="$2,$CY,0"
  echo
  echo "########## $1   CAMSHIFT=$CAMSHIFT   KSAMPOPEN=$KSAMPOPEN   dy=$3 x$4"
  TAG=$1 python3 $W/pos_grid.py --dx="${DXL:-0}" --dy="$3" --reps "$4" \
      --tag "$1" --out $W/exp_$1.csv --logdir $W/gridlogs
}

case "$MODE" in
  xsweep)
    run x168 "-0.0168" "0" 6      # 현재값 (대조군, 앞선 8판과 합산 가능)
    run x205 "-0.0205" "0" 6      # 목표 dx -24 mm
    run x233 "-0.0233" "0" 6      # 목표 dx -20 mm
    python3 $W/rep3.py 'x1*' 'x2*'
    for T in x168 x205 x233; do wilson $W/exp_$T.csv "CAMSHIFT x=$T"; done
    echo
    echo "dx 평균이 가장 0 에 가깝고 ok 가 높은 x 로:"
    echo "  bash tune6.sh final -0.0205"
    ;;
  final)
    X="${ARG:--0.0205}"
    run kfin "$X" "-0.05,0,0.05" 5
    python3 $W/rep3.py 'kfin'
    wilson $W/exp_kfin.csv "최종 x=$X, 캔 ±50 mm"
    ;;
  wide)
    X="${ARG:--0.0205}"
    export DXL="-0.06,0,0.06"
    run kwide "$X" "-0.06,0,0.06" 3
    python3 $W/rep3.py 'kwide'
    wilson $W/exp_kwide.csv "격자 ±60 mm, x=$X"
    ;;
  *) echo "usage: bash tune6.sh [xsweep|final <x>|wide <x>]"; exit 1 ;;
esac
