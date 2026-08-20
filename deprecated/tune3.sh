#!/usr/bin/env bash
# tune3.sh -- 조준은 됐다(dy 4.1 mm). 남은 것은 '물고 드는' 구간이다.
#
#   bash tune3.sh diag        기존 로그만 재분석. 시뮬 안 돌린다. 즉시
#   bash tune3.sh sweep       후보 6종 x 3판 (dy=0). 약 15분
#   bash tune3.sh final <조건>  이긴 조건으로 dy = -0.05/0/+0.05 x 4판
#
# 기준 설정은 17번 문서 §2 확정값. 여기서 바꾸는 것은 '닫기·유지' 계열뿐이다.
set -u
MODE="${1:-diag}"
W=/home/data/groot/work
export LOGDIR=$W/gridlogs

if [ "$MODE" = "diag" ]; then
  python3 $W/rep3.py
  exit 0
fi

cd /home/data/groot/MIGRATE/ENV
source /home/data/groot/venv-isaacsim/bin/activate
unset CUDA_VISIBLE_DEVICES MUJOCO_GL PYOPENGL_PLATFORM
export NOWIN=1 OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/data/groot/MIGRATE/ENV
export RUNNER=$W/run_groot31.py
export VIDROOT=$W/vid VIDFPS=20 VIDEVERY=2
mkdir -p "$VIDROOT"

# ---- 17번 §2 확정 설정 (건드리지 않는다) ----
export WCROP=1 WROT=0 WFLIP=0 FCROP=1
export SETTLE=8 RESEED=0 IDLE=1
export ASCALE=0.05 NSUB=8 ZBIAS=0 XBIAS=-0.19 YBIAS=0.105
export GLATCH=0 ARMONLY=1 LIFTTH=0.08 GDEB=3
export ZSCALE=1.0 GOFF=0.018 FLOORB=0.0 YMAX=0.106
export INSTR="pick up the tomato sauce and place it in the basket"
export ANCHOR=detect CANRAD=0.0329 CANRK=0.72 CANZOFF=-0.0077
export CAMKEY=wrist CAMMODE=wristdup
export CAMSHIFT=-0.0168,0.0283,0
export NSUBOPEN=2 MAXSTEPS=90
export FITDX=0.045 FITDY=0.0063 FITDZ=0.020
# ---- 여기서부터가 이번에 바꾸는 것 ----
export GHOLD=24 CLOSEMIN=4 ABORT=1

python3 $W/front_mode.py original || exit 1

run () {   # run <tag> <dylist> <reps>
  echo
  echo "########## $1"
  echo "  CLOSEMIN=$CLOSEMIN GHOLD=$GHOLD GDEB=$GDEB ASCALE=$ASCALE ABORT=$ABORT"
  TAG=$1 python3 $W/pos_grid.py --dx=0 --dy="$2" --reps "$3" \
      --tag "$1" --out $W/exp_$1.csv --logdir $W/gridlogs
}

case "$MODE" in
  sweep)
    # 후보 1-3: 닫기 시점.  성공 판은 close_step 5, 실패 판은 11 이었다.
    export CLOSEMIN=2; run t_cm2 "0" 3
    export CLOSEMIN=4; run t_cm4 "0" 3      # 기준선
    export CLOSEMIN=6; run t_cm6 "0" 3
    # 후보 4: 닫힘 유지를 늘린다 (들어올리는 동안 다시 열리는지)
    export CLOSEMIN=4 GHOLD=48; run t_gh48 "0" 3
    export GHOLD=24
    # 후보 5: 닫기 디바운스를 줄여 더 빨리 문다
    export GDEB=2; run t_gd2 "0" 3
    export GDEB=3
    # 후보 6: 접근 보폭 축소. 16번 §2-2 에서 재측정 대상으로 표시된 항목
    export ASCALE=0.035; run t_as035 "0" 3
    export ASCALE=0.05
    echo
    python3 $W/rep3.py 't_*'
    echo
    echo "이긴 조건으로:  bash tune3.sh final t_XXX"
    ;;

  final)
    C="${2:-t_cm4}"
    case "$C" in
      t_cm2)   export CLOSEMIN=2 ;;
      t_cm6)   export CLOSEMIN=6 ;;
      t_gh48)  export GHOLD=48 ;;
      t_gd2)   export GDEB=2 ;;
      t_as035) export ASCALE=0.035 ;;
      *)       : ;;
    esac
    run fin_"$C" "-0.05,0,0.05" 4
    python3 $W/rep3.py "fin_$C"
    ;;

  *) echo "usage: bash tune3.sh [diag|sweep|final <tag>]"; exit 1 ;;
esac
