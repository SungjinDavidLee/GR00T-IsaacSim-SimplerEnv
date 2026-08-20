#!/usr/bin/env bash
# pos_grid_one.sh -- 한 좌표에서 1판 실행. randtest.sh 가 호출한다.
#   bash pos_grid_one.sh <obj> <dx> <dy> <tag>
# 조준 상수(CANRK/AIMX/AIMY/AIMZ)는 호출한 쉘의 env 를 그대로 물려받는다.
set -u
OBJIN="${1:-can}"; DX="${2:-0}"; DY="${3:-0}"; TG="${4:-rnd}"
W=/home/data/groot/work
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
export ANCHOR=detect CAMKEY=wrist CAMMODE=wristdup
export CAMSHIFT=${CAMSHIFT:--0.0180,0.0283,0}
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1 NOREOPEN=0
export FITDX=0.045 FITDY=0.0063 FITDZ=0.025
export KSAMP=1 KSAMPOPEN=${K:-8} KLOG=0
export DETMODE=text DETMINSCORE=0.10 DETFALLBACK=0
export DETMAXFRAC=0.05 DETSHRINK=0.80 DETGEOM=ray

case "$OBJIN" in
  can)    export OBJ=can         DETPROMPT="a red soda can"
          export INSTR="pick up the coke can and lift it up"
          export CANRAD=0.0329 CANRK=${CANRK:-0.413} CANZOFF=${CANZOFF:--0.0416} ;;
  sponge) export OBJ=sponge      DETPROMPT="a green sponge"
          export INSTR="pick up the sponge and lift it up"
          export CANRAD=0.0329 CANRK=${CANRK:-0.040} CANZOFF=${CANZOFF:--0.0180} ;;
  bottle) export OBJ=blue_bottle DETPROMPT="a plastic water bottle"
          export INSTR="pick up the bottle and lift it up"
          export CANRAD=0.0329 CANRK=${CANRK:-0.375} CANZOFF=${CANZOFF:--0.0495} ;;
  *) echo "대상: can | sponge | bottle"; exit 1 ;;
esac

python3 $W/front_mode.py original >/dev/null || exit 1
TAG=$TG python3 $W/pos_grid.py --dx="$DX" --dy="$DY" --reps 1 \
    --tag "$TG" --out $W/exp_$TG.csv --logdir $W/gridlogs
