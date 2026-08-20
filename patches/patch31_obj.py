#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch31_obj.py -- run_groot31.py 의 평가 대상 물체를 env 로 바꾼다.

 지금은 `find_obj(backend, "can")` 이 6곳에 하드코딩되어 있다.
 그래서 스펀지를 잡으러 가도 오차·성공 판정은 **캔 기준**으로 계산된다.
 (파지 자체는 ANCHOR=detect 라 검출 좌표로 가므로 동작은 한다. 지표만 무의미해진다.)

 OBJ=sponge  로 두면 평가·판정·전도감지(ABORT)·PERTURB 가 전부 그 물체를 본다.
 검출 프롬프트(DETPROMPT)와 반드시 짝을 맞출 것.

   OBJ=can          DETPROMPT="a red soda can"            CANRAD=0.0329
   OBJ=sponge       DETPROMPT="a green sponge"            CANRAD=0.035
   OBJ=blue_bottle  DETPROMPT="a plastic water bottle"    CANRAD=0.032

 사용: python3 patch31_obj.py [work_dir]
"""
import os, sys, py_compile, shutil, re

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch31_obj: OBJ ---"

DEF_OLD = '''def find_obj(backend, pat):'''
DEF_NEW = MARK + '''
OBJNAME = os.environ.get("OBJ", "can")


def find_obj(backend, pat):'''


def main():
    p = os.path.join(WORK, "run_groot31.py")
    s = open(p).read()
    if MARK in s:
        print("[patch31_obj] 이미 패치됨"); return
    if s.count(DEF_OLD) != 1:
        raise SystemExit("find_obj 정의 앵커를 못 찾음. 중단.")
    s = s.replace(DEF_OLD, DEF_NEW)
    n1 = len(re.findall(r'find_obj\(backend, "can"\)', s))
    s = re.sub(r'find_obj\(backend, "can"\)', 'find_obj(backend, OBJNAME)', s)
    n2 = len(re.findall(r'backend\.objects\["can"\]', s))
    s = re.sub(r'backend\.objects\["can"\]', 'backend.objects[OBJNAME]', s)
    s = s.replace('backend.objects[\'can\']', 'backend.objects[OBJNAME]')
    shutil.copyfile(p, p + ".bak_obj")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("[patch31_obj] find_obj %d곳, backend.objects %d곳 치환" % (n1, n2))
    print("[patch31_obj] ok  (백업: %s.bak_obj)" % p)


if __name__ == "__main__":
    main()
