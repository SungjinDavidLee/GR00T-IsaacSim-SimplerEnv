#!/usr/bin/env bash
# pick3.sh v2 -- 물체 상수 + 조준 보정으로 세 물체를 잡는다.
#
# v2 수정: GHOLD / NOREOPEN 을 스크립트가 덮어써서 호출 시 지정한 값이
#          무시되던 결함을 고쳤다. 이제 앞에 붙인 env 가 우선한다.
#
#   bash pick3.sh can      캔 3판
#   bash pick3.sh sponge   스펀지 3판
#   bash pick3.sh bottle   물병 3판
#   bash pick3.sh check <obj>   검출만 1판 (오차 0 근처인지 확인, 20초)
#
# 상수 출처: 2026-08-19 run_all.sh geom 의 'CANDET calib' 줄 (OWL-v2 박스 마스크 기준)
#   can     k_best 0.413  zoff_best -0.0416
#   sponge  k_best 0.040  zoff_best -0.0180
#   bottle  k_best 0.375  zoff_best -0.0495
#
# 이전 값(캔 k=0.72 zoff=-0.0077)과 다른 이유: HSV 빨강 마스크는 캔 몸통만 덮었고,
# OWL-v2 박스는 은색 뚜껑까지 포함한다. 마스크가 달라지면 표면 중심이 달라진다.
# **마스크 방식을 바꾸면 상수를 다시 잡아야 한다.**
set -u
OBJIN="${1:-can}"
REPS="${2:-3}"
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
export GLATCH=0 ARMONLY=1 LIFTTH=0.08 GDEB=3
export GHOLD=${GHOLD:-24}
export ZSCALE=1.0 GOFF=0.018 YMAX=0.106 FLOORB=0.0
export ANCHOR=detect CAMKEY=wrist CAMMODE=wristdup
export CAMSHIFT=${CAMSHIFT:--0.0180,0.0283,0}
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1
export NOREOPEN=${NOREOPEN:-0}
export FITDX=0.045 FITDY=0.0063 FITDZ=0.025
export KSAMP=1 KSAMPOPEN=${K:-8} KLOG=0
export DETMODE=text DETMINSCORE=0.10 DETFALLBACK=0
export DETMAXFRAC=0.05 DETSHRINK=0.80
export DETGEOM=ray            # 상수 방식. DETGEOM=top 은 패치 적용 후에

case "$OBJIN" in
  can)
    export OBJ=can DETPROMPT="a red soda can"
    export INSTR="pick up the coke can and lift it up"
    export CANRAD=0.0329 CANRK=${CANRK:-0.413} CANZOFF=${CANZOFF:--0.0416} ;;
  sponge)
    export OBJ=sponge DETPROMPT="a green sponge"
    export INSTR="pick up the sponge and lift it up"
    export CANRAD=0.0329 CANRK=${CANRK:-0.040} CANZOFF=${CANZOFF:--0.0180} ;;
  bottle)
    export OBJ=blue_bottle DETPROMPT="a plastic water bottle"
    export INSTR="pick up the bottle and lift it up"
    export CANRAD=0.0329 CANRK=${CANRK:-0.375} CANZOFF=${CANZOFF:--0.0495} ;;
  check)
    exec env DUMMY=1 bash "$0" "${2:-can}" check ;;
  *) echo "대상: can | sponge | bottle"; exit 1 ;;
esac

echo "=============================================="
echo " $OBJ  |  \"$DETPROMPT\""
echo " CANRAD=$CANRAD CANRK=$CANRK CANZOFF=$CANZOFF"
echo "=============================================="
python3 $W/front_mode.py original || exit 1

if [ "$REPS" = "check" ]; then
  export MAXSTEPS=1
  python -u $W/run_groot31.py --config config/environment_groot.yaml 2>&1 \
    | grep -E "^\[obj_detect\] 채택|^CANDET"
  echo
  echo "'CANDET err' 의 dx dy dz 가 모두 0.005 이하면 상수가 맞은 것이다."
  exit 0
fi

TAG=k_$OBJ python3 $W/pos_grid.py --dx=0 --dy=0 --reps "$REPS" \
    --tag "k_$OBJ" --out $W/exp_k_$OBJ.csv --logdir $W/gridlogs
python3 $W/rep3.py "k_$OBJ"
