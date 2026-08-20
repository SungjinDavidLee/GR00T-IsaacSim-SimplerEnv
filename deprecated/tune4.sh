#!/usr/bin/env bash
# tune4.sh -- 한 번에 문 판은 들었다. 물었다 놓는 것을 막는다.
#
#   bash tune4.sh sweep      NOREOPEN A/B 4조건 x 4판 (dy=0). 약 12분
#   bash tune4.sh final <조건>  이긴 조건으로 dy = -0.05/0/+0.05 x 4판  ← 최종
#   bash tune4.sh wide  <조건>  dx,dy 를 ±0.06 까지 (시연용)
#
# 근거: tune3 18판에서 성공 3판은 전부 nclose=1, 실패 15판은 평균 2.67.
#       fit=1 인데 못 든 3판은 nclose 4/3/2, abort=0, nstep=90 — 재시도만 하다 끝났다.
set -u
MODE="${1:-sweep}"
COND="${2:-}"
W=/home/data/groot/work
export LOGDIR=$W/gridlogs
cd /home/data/groot/MIGRATE/ENV
source /home/data/groot/venv-isaacsim/bin/activate
unset CUDA_VISIBLE_DEVICES MUJOCO_GL PYOPENGL_PLATFORM
export NOWIN=1 OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/data/groot/MIGRATE/ENV
export RUNNER=$W/run_groot31.py
export VIDROOT=$W/vid VIDFPS=20 VIDEVERY=2
mkdir -p "$VIDROOT"

# ---- 17번 §2 확정 설정 ----
export WCROP=1 WROT=0 WFLIP=0 FCROP=1
export SETTLE=8 RESEED=0 IDLE=1
export ASCALE=0.05 NSUB=8 ZBIAS=0 XBIAS=-0.19 YBIAS=0.105
export GLATCH=0 ARMONLY=1 LIFTTH=0.08 GDEB=3
export ZSCALE=1.0 GOFF=0.018 FLOORB=0.0 YMAX=0.106
export INSTR="pick up the tomato sauce and place it in the basket"
export ANCHOR=detect CANRAD=0.0329 CANRK=0.72 CANZOFF=-0.0077
export CAMKEY=wrist CAMMODE=wristdup
export CAMSHIFT=-0.0168,0.0283,0
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1
export FITDX=0.045 FITDY=0.0063 FITDZ=0.020

python3 $W/front_mode.py original || exit 1

run () {   # run <tag> <dylist> <reps>
  echo
  echo "########## $1   NOREOPEN=${NOREOPEN:-0} GHOLD=$GHOLD CLOSEMIN=$CLOSEMIN"
  TAG=$1 python3 $W/pos_grid.py --dx="${DXL:-0}" --dy="$2" --reps "$3" \
      --tag "$1" --out $W/exp_$1.csv --logdir $W/gridlogs
}

setcond () {
  case "$1" in
    u_h24)     export NOREOPEN=1 GHOLD=24 ;;
    u_h48)     export NOREOPEN=1 GHOLD=48 ;;
    u_h48s)    export NOREOPEN=1 GHOLD=48 SETTLE=12 ;;
    u_off48)   export NOREOPEN=0 GHOLD=48 ;;   # tune3 재현 대조군
    *) echo "알 수 없는 조건: $1"; exit 1 ;;
  esac
}

case "$MODE" in
  sweep)
    for C in u_off48 u_h24 u_h48 u_h48s; do
      ( setcond $C; run $C "0" 4 )
    done
    python3 $W/rep3.py 'u_*'
    echo
    echo "이긴 조건으로:  bash tune4.sh final u_XXX"
    ;;
  final)
    [ -n "$COND" ] || { echo "조건을 지정하세요: bash tune4.sh final u_h48"; exit 1; }
    setcond "$COND"
    run "f_$COND" "-0.05,0,0.05" 4
    python3 $W/rep3.py "f_$COND"
    ;;
  wide)
    [ -n "$COND" ] || { echo "조건을 지정하세요"; exit 1; }
    setcond "$COND"
    export DXL="-0.06,0,0.06"
    run "w_$COND" "-0.06,-0.03,0,0.03,0.06" 2
    python3 $W/rep3.py "w_$COND"
    ;;
  *) echo "usage: bash tune4.sh [sweep|final <cond>|wide <cond>]"; exit 1 ;;
esac
