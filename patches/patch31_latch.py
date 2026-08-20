#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch31_latch.py -- run_groot31.py 에 NOREOPEN 을 추가한다.

 근거 (2026-08-18 tune3 sweep, 18판):
   성공 3판   nclose = 1, 1, 1        nstep 평균 13.7
   실패 15판  nclose 평균 2.67        nstep 평균 56.9
   fit=1 인데 못 든 3판  nclose = 4, 3, 2,  전부 abort=0, nstep=90

   **한 번에 문 판은 들었고, 물었다 놓고 다시 시도한 판은 하나도 못 들었다.**
   16번 §10-4의 '극한주기'가 파지 성공을 직접 갉아먹고 있다.

 현재 코드는 `hold` 가 소진되면 모델의 열기 명령을 그대로 따른다:
     elif (not grip_open) and g > 0.3:   -> set_gripper(True)

 NOREOPEN=1 이면 CLOSEMIN 이후의 첫 파지가 성립한 뒤로는 열기 명령을 무시한다.
 실물에서도 타당한 규약이다 — 물체를 문 다음에는 놓지 않는다.

 참고: `GLATCH` 는 헤더에 출력만 되고 **어디에서도 사용되지 않는다**(사문화된 env).
       NOREOPEN 이 그 자리를 대신한다.

 사용: python3 patch31_latch.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch31_latch: NOREOPEN ---"

DEF_OLD = """    GHOLD = int(os.environ.get('GHOLD', '0'))"""
DEF_NEW = """    GHOLD = int(os.environ.get('GHOLD', '0'))
    """ + MARK + """
    NOREOPEN = int(os.environ.get('NOREOPEN', '0'))
    latched = [False]
    print("NOREOPEN", NOREOPEN, "(1이면 첫 파지 후 열기 명령 무시)")"""

CLOSE_OLD = """                grip_open = False
                hold = GHOLD
                set_gripper(backend, robot, False)"""
CLOSE_NEW = """                grip_open = False
                hold = GHOLD
                if NOREOPEN:
                    latched[0] = True          # """ + MARK + """
                set_gripper(backend, robot, False)"""

OPEN_OLD = """            elif (not grip_open) and g > 0.3:
                grip_open = True
                set_gripper(backend, robot, True)
                print("GRIP OPEN  s%03d k%d" % (step, k))"""
OPEN_NEW = """            elif (not grip_open) and g > 0.3 and not latched[0]:
                grip_open = True
                set_gripper(backend, robot, True)
                print("GRIP OPEN  s%03d k%d" % (step, k))
            elif (not grip_open) and g > 0.3 and latched[0] and k == 0:
                print("GRIP OPEN ignored s%03d (NOREOPEN)" % step)"""


def sub(s, old, new, what):
    n = s.count(old)
    if n != 1:
        raise SystemExit("[patch31_latch] '%s' 앵커 %d 곳 (1이어야 함). 중단." % (what, n))
    print("  ok:", what)
    return s.replace(old, new)


def main():
    p = os.path.join(WORK, "run_groot31.py")
    s = open(p).read()
    if MARK in s:
        print("[patch31_latch] 이미 패치됨 — 건너뜀"); return
    s = sub(s, DEF_OLD, DEF_NEW, "NOREOPEN 정의")
    s = sub(s, CLOSE_OLD, CLOSE_NEW, "파지 시 latch")
    s = sub(s, OPEN_OLD, OPEN_NEW, "열기 명령 차단")
    shutil.copyfile(p, p + ".bak_latch")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("  SYNTAX OK  (백업: %s.bak_latch)" % p)


if __name__ == "__main__":
    main()
