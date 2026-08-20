#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch31_zero.py -- run_groot31.py 에 ZEROACT 를 추가한다. (대조 실험 전용)

 목적: "이 파지는 정말 GR00T 의 예측 행동인가, 아니면 우리가 짜 넣은 홈잉·클램프가
       하고 있는 것인가"를 한 판으로 판정한다.

   ZEROACT=1   정책의 **병진 출력을 0으로** 만든다 (그리퍼 명령은 유지)
   ZEROACT=2   정책 출력을 **무작위**로 대체 (같은 크기의 잡음)

 판정:
   ZEROACT=1 에서도 팔이 물체로 내려가 잡는다  -> 홈잉/클램프가 하고 있다. 결과 무효.
   ZEROACT=1 에서 제자리에 머물고 실패한다      -> 접근·파지는 GR00T 의 행동이다. [확인]

 이 실험 결과는 보고서에 그대로 들어갈 근거다. 성능 개선용이 아니다.

 사용: python3 patch31_zero.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch31_zero: ZEROACT ---"

OLD = """        ch = np.stack(["""
NEW = """        """ + MARK + """
        _za = int(os.environ.get("ZEROACT", "0"))
        ch = np.stack(["""

OLD2 = """        _c3 = ch[:, :3].copy()"""
NEW2 = """        if _za == 1:                     # """ + MARK + """
            ch[:, :3] = 0.0
            if step == 0:
                print("ZEROACT=1  정책 병진 출력을 0으로 (대조 실험)")
        elif _za == 2:
            _rs = np.random.default_rng(1234 + step)
            _sd = float(np.abs(ch[:, :3]).mean()) + 1e-9
            ch[:, :3] = _rs.normal(0.0, _sd, size=ch[:, :3].shape)
            if step == 0:
                print("ZEROACT=2  정책 병진 출력을 무작위로 대체 (대조 실험)")
        _c3 = ch[:, :3].copy()"""


def main():
    p = os.path.join(WORK, "run_groot31.py")
    s = open(p).read()
    if MARK in s:
        print("[patch31_zero] 이미 패치됨"); return
    for old, what in ((OLD, "ZEROACT 읽기"), (OLD2, "chunk 치환")):
        if s.count(old) != 1:
            raise SystemExit("[patch31_zero] '%s' 앵커 %d 곳 (1이어야 함). 중단."
                             % (what, s.count(old)))
    s = s.replace(OLD, NEW).replace(OLD2, NEW2)
    shutil.copyfile(p, p + ".bak_zero")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("[patch31_zero] ok  (백업: %s.bak_zero)" % p)


if __name__ == "__main__":
    main()
