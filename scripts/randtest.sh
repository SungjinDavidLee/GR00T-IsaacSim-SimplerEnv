#!/usr/bin/env bash
# randtest.sh -- 물체를 무작위 위치에 놓고 잡는지 본다. 격자가 아니라 임의 좌표다.
#
#   bash randtest.sh can 8         캔을 ±60 mm 안 임의 위치 8곳, 각 1판
#   bash randtest.sh sponge 8
#   bash randtest.sh bottle 8
#   SEED=7 RANGE=0.08 bash randtest.sh can 10    범위·시드 변경
#
# 격자 스윕과 다른 점: 미리 정한 점이 아니라 매번 다른 좌표라서
# "튜닝한 지점에서만 되는 것"인지 아닌지가 드러난다.
#
# 조준 상수는 pick3.sh 와 동일하게 env 로 넘긴다. 예)
#   CANRK=0.413 AIMX=0.0415 AIMY=0.0165 bash randtest.sh can 8
set -u
OBJIN="${1:-can}"
N="${2:-8}"
SEED="${SEED:-1}"
RANGE="${RANGE:-0.06}"
W=/home/data/groot/work
export LOGDIR=$W/gridlogs

PAIRS=$(python3 - "$N" "$SEED" "$RANGE" <<'EOF'
import random, sys
n, seed, rng = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
r = random.Random(seed)
for _ in range(n):
    print("%.4f %.4f" % (r.uniform(-rng, rng), r.uniform(-rng, rng)))
EOF
)

OUT=$W/exp_rand_$OBJIN.csv
i=0
echo "tag,dx,dy,rep,ok,fit,close_step,err_dx,err_dy,err_dz,lift,abort,nstep,canx,cany,clampx,clampy,nclose,last_dy,cammode,frontpos,log,vid,det_dx,det_dy,det_dz,anchor" > $OUT.head
: > $OUT.rows

while read -r DX DY; do
  [ -z "$DX" ] && continue
  i=$((i+1))
  echo
  echo "########## 무작위 $i/$N   dx=$DX  dy=$DY"
  TAGR=rnd${i}_$OBJIN
  TAG=$TAGR bash $W/pos_grid_one.sh "$OBJIN" "$DX" "$DY" "$TAGR" || true
done <<< "$PAIRS"

echo
echo "== 무작위 위치 결과 =="
python3 $W/rep3.py "rnd*_$OBJIN"
python3 - "$W" "$OBJIN" <<'EOF'
import csv, glob, math, sys, os
W, obj = sys.argv[1], sys.argv[2]
k = n = 0
for f in glob.glob(os.path.join(W, "exp_rnd*_%s.csv" % obj)):
    for r in csv.DictReader(open(f)):
        n += 1; k += int(r["ok"])
if n:
    z = 1.96; p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    print("\n무작위 위치 성공률: %d/%d = %.0f%%   95%% 구간 %.0f~%.0f%%"
          % (k, n, 100 * p, 100 * max(0, c - h), 100 * min(1, c + h)))
    print("이 값이 고정 위치 성공률(캔 3/3)과 비슷하면 위치 일반화가 된 것이다.")
EOF
