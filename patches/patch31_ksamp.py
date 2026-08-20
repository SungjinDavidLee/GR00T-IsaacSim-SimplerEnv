#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch31_ksamp.py -- run_groot31.py 에 KSAMP (다중 샘플 중앙값) 를 추가한다.

 확정된 사실 (POLICYCHK, 2026-08-18):
   **같은 관측으로 정책을 5번 불렀더니 매번 다른 행동이 나왔다.**
     dim=112,  std 평균 0.0136 / 최대 0.067,  범위 최대 0.189
   GR00T N1.7 의 action head 는 flow matching 계열이라 호출마다 노이즈를 새로 뽑는다.
   ASCALE=0.05 를 곱하면 substep 당 최대 9.5 mm, 청크 8스텝이면 수 cm 가 흔들린다.
   → 관측한 파지 dx 산포 20~40 mm 와 크기가 맞는다.

 왜 시드 고정이 아니라 중앙값인가:
   시드를 고정하면 **재현은 되지만 좋아지지는 않는다.** 임의의 한 표본에 갇힐 뿐이다.
   같은 관측에서 K개를 뽑아 중앙값을 쓰면 샘플링 잡음이 약 sqrt(K) 배 줄고,
   기대 행동에 가까워진다. 모델을 수정하지 않으며 파인튜닝이 아니다.
   실물에서도 그대로 쓸 수 있다(추론 K배 비용만 든다).

 KSAMP=K        모든 스텝에서 K개 샘플의 중앙값 사용 (기본 1 = 기존 동작)
 KSAMPOPEN=K    그리퍼가 열려 있는 접근 구간에만 별도 K (정밀도가 필요한 구간)
 KLOG=1         스텝마다 샘플 간 표준편차를 출력 (분산 감소 확인용)

 사용: python3 patch31_ksamp.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch31_ksamp: K-sample median ---"

OLD = """        act = client.get_action(gobs)"""
NEW = """        """ + MARK + """
        _K = int(os.environ.get("KSAMP", "1"))
        if grip_open:
            _K = int(os.environ.get("KSAMPOPEN", str(_K)))
        if _K > 1:
            import copy as _cp
            _samples = [client.get_action(_cp.deepcopy(gobs)) for _ in range(_K)]
            act = {}
            for _k in _samples[0]:
                try:
                    _st = np.stack([np.asarray(_s[_k], dtype=np.float64)
                                    for _s in _samples])
                except Exception:
                    act[_k] = _samples[0][_k]
                    continue
                act[_k] = np.median(_st, axis=0)
                if int(os.environ.get("KLOG", "0")) and step < 6:
                    print("KSAMP s%03d %-16s K=%d std %.5f -> 중앙값 잔여 %.5f"
                          % (step, str(_k), _K, _st.std(axis=0).mean(),
                             _st.std(axis=0).mean() / max(_K ** 0.5, 1.0)))
        else:
            act = client.get_action(gobs)"""


def main():
    p = os.path.join(WORK, "run_groot31.py")
    s = open(p).read()
    if MARK in s:
        print("[patch31_ksamp] 이미 패치됨 — 건너뜀"); return
    n = s.count(OLD)
    if n != 1:
        raise SystemExit("[patch31_ksamp] 앵커 %d 곳 (1이어야 함). "
                         "patch31_det2 를 먼저 적용했는지 확인." % n)
    s = s.replace(OLD, NEW)
    shutil.copyfile(p, p + ".bak_ks")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("[patch31_ksamp] ok  (백업: %s.bak_ks)" % p)


if __name__ == "__main__":
    main()
