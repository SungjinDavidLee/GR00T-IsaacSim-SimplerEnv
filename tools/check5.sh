#!/usr/bin/env bash
# check5.sh -- 조건을 더 바꾸기 전에, 측정이 성립하는지부터 확인한다.
#
#   bash check5.sh policy    정책이 결정적인가 (1판, 약 1분)          ← 먼저
#   bash check5.sh sim       시뮬이 결정적인가 (같은 설정 3판 비교)
#   bash check5.sh floor     현재 최선 설정의 진짜 성공률 (dy=0 x 16판, 약 20분)
#
# 왜: 같은 설정이 1/3 -> 0/4 로 나왔다. 조건을 바꾸지 않았는데 결과가 달라진다.
#     n=3~4 판으로는 어떤 조건도 구분되지 않는다 (전 조건 95% 구간이 서로 겹침).
#     지금까지의 "이 값이 낫다" 판정은 전부 잡음일 수 있다.
set -u
MODE="${1:-policy}"
W=/home/data/groot/work
export LOGDIR=$W/gridlogs
cd /home/data/groot/MIGRATE/ENV
source /home/data/groot/venv-isaacsim/bin/activate
unset CUDA_VISIBLE_DEVICES MUJOCO_GL PYOPENGL_PLATFORM
export NOWIN=1 OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/data/groot/MIGRATE/ENV
export RUNNER=$W/run_groot31.py
export VIDROOT=$W/vid VIDFPS=20 VIDEVERY=2
mkdir -p "$VIDROOT"

# ---- 17번 §2 확정 설정 + tune3 최선 ----
export WCROP=1 WROT=0 WFLIP=0 FCROP=1
export SETTLE=8 RESEED=0 IDLE=1
export ASCALE=0.05 NSUB=8 ZBIAS=0 XBIAS=-0.19 YBIAS=0.105
export GLATCH=0 ARMONLY=1 LIFTTH=0.08 GDEB=3 GHOLD=24
export ZSCALE=1.0 GOFF=0.018 FLOORB=0.0 YMAX=0.106
export INSTR="pick up the tomato sauce and place it in the basket"
export ANCHOR=detect CANRAD=0.0329 CANRK=0.72 CANZOFF=-0.0077
export CAMKEY=wrist CAMMODE=wristdup CAMSHIFT=-0.0168,0.0283,0
export NSUBOPEN=2 MAXSTEPS=90 CLOSEMIN=4 ABORT=1 NOREOPEN=0
export FITDX=0.045 FITDY=0.0063 FITDZ=0.020

python3 $W/front_mode.py original || exit 1

case "$MODE" in
  policy)
    # 같은 관측으로 정책을 5번 호출해 반환 청크를 비교한다.
    export POLICYCHK=5 MAXSTEPS=1
    L=$W/policychk.log
    python -u $W/run_groot31.py --config config/environment_groot.yaml 2>&1 \
      | tee $L | grep -E "^POLICYCHK|^CANDET|^RESULT"
    echo
    echo "판정 줄을 보세요. '확률적' 이면 서버에서 시드를 고정해야"
    echo "조건 비교가 성립합니다. '결정적' 이면 변동원은 시뮬레이터입니다."
    ;;

  sim)
    # 완전히 같은 설정 3판. 첫 파지 dx/dy 가 얼마나 벌어지는지가 잡음 하한이다.
    export TAG=simchk
    python3 $W/pos_grid.py --dx=0 --dy=0 --reps 3 \
      --tag simchk --out $W/exp_simchk.csv --logdir $W/gridlogs
    python3 $W/rep3.py 'simchk'
    ;;

  floor)
    # 조건을 바꾸지 않고 판수만 늘려 진짜 성공률을 낸다.
    python3 $W/pos_grid.py --dx=0 --dy=0 --reps 16 \
      --tag base16 --out $W/exp_base16.csv --logdir $W/gridlogs
    python3 $W/rep3.py 'base16'
    python3 - <<'EOF'
import csv, math
r = list(csv.DictReader(open('/home/data/groot/work/exp_base16.csv')))
k = sum(int(x['ok']) for x in r); n = len(r)
z = 1.96; p = k/n if n else 0; d = 1+z*z/n
c = (p+z*z/(2*n))/d
h = z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
print("\n진짜 기준선: %d/%d = %.0f%%   95%% 구간 %.0f~%.0f%%"
      % (k, n, 100*p, 100*max(0,c-h), 100*min(1,c+h)))
print("앞으로 어떤 조건도 이 구간을 벗어나야 '나아졌다'고 말할 수 있다.")
EOF
    ;;
  *) echo "usage: bash check5.sh [policy|sim|floor]"; exit 1 ;;
esac
