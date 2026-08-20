#!/usr/bin/env bash
# lang_test.sh -- "자연어 명령이 실제로 작동하는가" 를 측정한다.
#
#   bash lang_test.sh instr     지시문 5종 x 3판 (검출은 캔 고정). 약 20분
#   bash lang_test.sh object    같은 지시문으로 검출 대상만 바꾼다 (캔 / 스펀지)
#
# 왜 이걸 재야 하는가:
#   과제가 요구하는 것은 "자연어 명령 -> 대상 지목 -> 좌표 파악 -> 파지" 다.
#   지금 구조에서 대상 지목과 좌표는 can_detect.py(HSV 빨강)가 하고 있고,
#   GR00T 에게 주는 INSTR 은 "pick up the tomato sauce..." 로 캔을 언급조차 안 한다.
#   그런데도 캔을 잡는다. => 언어가 실제로 쓰이는지 검증된 적이 없다.
#
#   이 실험이 답을 준다:
#     지시문을 바꿔도 dx/dy/성공률이 안 변한다  -> 언어는 무시되고 있다 (구조 변경 필요)
#     지시문에 따라 유의하게 변한다             -> 언어 경로가 살아 있다 (활용 가능)
set -u
MODE="${1:-instr}"
W=/home/data/groot/work
export LOGDIR=$W/gridlogs
cd /home/data/groot/MIGRATE/ENV
source /home/data/groot/venv-isaacsim/bin/activate
unset CUDA_VISIBLE_DEVICES MUJOCO_GL PYOPENGL_PLATFORM
export NOWIN=1 OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/data/groot/MIGRATE/ENV
export RUNNER=$W/run_groot31.py
export VIDROOT=$W/vid VIDFPS=20 VIDEVERY=4
mkdir -p "$VIDROOT"

# ---- 현재 최선 설정 (K=8, x=-0.0180) ----
export WCROP=1 WROT=0 WFLIP=0 FCROP=1
export SETTLE=8 RESEED=0 IDLE=1
export ASCALE=0.05 NSUB=8 ZBIAS=0 XBIAS=-0.19 YBIAS=0.105
export GLATCH=0 ARMONLY=1 LIFTTH=0.08 GDEB=3 GHOLD=24
export ZSCALE=1.0 GOFF=0.018 YMAX=0.106 FLOORB=0.0
export ANCHOR=detect CANRAD=0.0329 CANRK=0.72 CANZOFF=-0.0077
export CAMKEY=wrist CAMMODE=wristdup CAMSHIFT=-0.0180,0.0283,0
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1 NOREOPEN=0
export FITDX=0.045 FITDY=0.0063 FITDZ=0.025
export KSAMP=1 KSAMPOPEN=${K:-8} KLOG=0

python3 $W/front_mode.py original || exit 1

case "$MODE" in
  instr)
    i=0
    while IFS= read -r I; do
      [ -z "$I" ] && continue
      i=$((i+1))
      export INSTR="$I"
      T=$(printf "L%d" $i)
      echo
      echo "########## $T   INSTR = \"$I\""
      TAG=$T python3 $W/pos_grid.py --dx=0 --dy=0 --reps 3 \
          --tag "$T" --out $W/exp_$T.csv --logdir $W/gridlogs
    done <<'EOF'
pick up the tomato sauce and place it in the basket
pick up the coke can and lift it up
pick up the can
pick up the sponge
do nothing
EOF
    python3 $W/rep3.py 'L1' 'L2' 'L3' 'L4' 'L5'
    echo
    echo "판정: 다섯 조건의 dx/dy/성공률이 서로 구분되지 않으면"
    echo "      **GR00T 는 지시문을 쓰지 않고 있다.** 특히 L5('do nothing') 에서도"
    echo "      똑같이 잡으면 언어 경로가 사실상 끊겨 있다는 결정적 증거다."
    ;;
  object)
    # 검출 대상을 스펀지(초록)로 바꾼다. 지시문은 캔을 말한다.
    # 정책이 프레임을 따라가는지, 아니면 캔을 고집하는지 본다.
    export INSTR="pick up the coke can and lift it up"
    export HUE="35,85,35,85" SMIN=40 VMIN=40 CANRAD=0.05 CANRK=0.7 CANZOFF=0
    echo "########## O1  검출=스펀지(초록), 지시문=캔"
    TAG=O1 python3 $W/pos_grid.py --dx=0 --dy=0 --reps 3 \
        --tag O1 --out $W/exp_O1.csv --logdir $W/gridlogs
    python3 $W/rep3.py 'O1'
    echo
    echo "주의: 이 모드는 CANRAD/CANZOFF 가 스펀지용으로 교정되지 않았다."
    echo "      성공률이 아니라 '팔이 어느 물체로 가는가'만 영상으로 볼 것."
    ;;
  *) echo "usage: bash lang_test.sh [instr|object]"; exit 1 ;;
esac
