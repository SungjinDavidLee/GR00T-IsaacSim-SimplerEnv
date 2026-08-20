#!/usr/bin/env bash
# tune5.sh -- 정책 샘플링 잡음을 K-샘플 중앙값으로 줄인다.
#
#   bash tune5.sh k8      KSAMP=8, dy=0 x 8판   (기준선 4/37=11% 와 비교)
#   bash tune5.sh k16     KSAMP=16, dy=0 x 6판  (k8 이 부족하면)
#   bash tune5.sh final   이긴 K 로 dy = -0.05/0/+0.05 x 5판     ← 최종
#   bash tune5.sh wide    dx,dy ±0.06 격자 (시연용)
#
#   K= 로 덮어쓸 수 있다:  K=12 bash tune5.sh k8
#
# 근거: POLICYCHK 결과 정책은 확률적이다 (std 0.0136, 범위 0.189).
#       K개 샘플의 중앙값을 쓰면 잡음이 약 sqrt(K) 배 줄어든다.
#       파인튜닝이 아니다. 모델도 관측도 건드리지 않는다.
set -u
MODE="${1:-k8}"
W=/home/data/groot/work
export LOGDIR=$W/gridlogs
cd /home/data/groot/MIGRATE/ENV
source /home/data/groot/venv-isaacsim/bin/activate
unset CUDA_VISIBLE_DEVICES MUJOCO_GL PYOPENGL_PLATFORM
export NOWIN=1 OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/data/groot/MIGRATE/ENV
export RUNNER=$W/run_groot31.py
export VIDROOT=$W/vid VIDFPS=20 VIDEVERY=4
mkdir -p "$VIDROOT"

# ---- 17번 §2 확정 설정 + tune3 최선 (t_cm4) ----
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
export KLOG=1

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
print("\n== %s ==  %d/%d = %.0f%%   95%% 구간 %.0f~%.0f%%" % (sys.argv[2], k, n, 100 * p, lo, hi))
print("   기준선(K=1) 4/37 = 11%%, 구간 4~25%%")
print("   판정: %s" % ("**개선 확정** (구간이 25%% 위에서 시작)" if lo > 25 else
                       "구간이 겹친다 — 판수를 더 늘리거나 다른 레버 필요"))
EOF
}

run () {   # run <tag> <dylist> <reps>
  echo
  echo "########## $1  KSAMP=${KSAMP:-1} KSAMPOPEN=${KSAMPOPEN:-$KSAMP}"
  TAG=$1 python3 $W/pos_grid.py --dx="${DXL:-0}" --dy="$2" --reps "$3" \
      --tag "$1" --out $W/exp_$1.csv --logdir $W/gridlogs
}

case "$MODE" in
  k8)
    export KSAMP=1 KSAMPOPEN=${K:-8}      # 접근 구간에만 K샘플. 파지 후엔 1
    run k8 "0" 8
    python3 $W/rep3.py 'k8'
    wilson $W/exp_k8.csv "KSAMPOPEN=${K:-8}"
    ;;
  k16)
    export KSAMP=1 KSAMPOPEN=${K:-16}
    run k16 "0" 6
    python3 $W/rep3.py 'k16'
    wilson $W/exp_k16.csv "KSAMPOPEN=${K:-16}"
    ;;
  final)
    export KSAMP=1 KSAMPOPEN=${K:-8}
    run kfin "-0.05,0,0.05" 5
    python3 $W/rep3.py 'kfin'
    wilson $W/exp_kfin.csv "최종 KSAMPOPEN=${K:-8}, 캔 ±50mm"
    ;;
  wide)
    export KSAMP=1 KSAMPOPEN=${K:-8} DXL="-0.06,0,0.06"
    run kwide "-0.06,0,0.06" 3
    python3 $W/rep3.py 'kwide'
    wilson $W/exp_kwide.csv "격자 ±60mm"
    ;;
  *) echo "usage: bash tune5.sh [k8|k16|final|wide]"; exit 1 ;;
esac
