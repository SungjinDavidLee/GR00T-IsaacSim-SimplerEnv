#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_candetect_aim.py -- can_detect.py 에 AIMX / AIMY / AIMZ 를 추가한다.

 왜 필요한가:
   검출 좌표는 이제 정확하다 (캔 err 0.2 mm). 남은 오차는 **정책의 접근 편향**이다.
   지금까지는 CANRK 를 올려 전방 보정을 했는데, CANRK 는 '카메라→물체 시선 방향'
   으로 미는 값이라 **물체가 화면 중앙에서 벗어나면 y 로도 밀린다.**
   물병(y=+0.10)이 갑자기 옆 허공을 집은 원인이 이것이다.

   AIM 은 시선과 무관하게 **로봇 베이스 프레임 축으로만** 조준점을 옮긴다.
   물체 위치와 무관하게 같은 방향으로 작용한다.

 부호 규약 (RbT = diag(-1,-1,1) 기준. 첫 판으로 반드시 확인할 것):
   AIMX +  : 로봇에서 **멀어지는** 쪽 (앞으로)
   AIMY -  : 로봇 기준 **오른쪽**
   AIMY +  : 로봇 기준 **왼쪽**
   AIMZ +  : 위로,  AIMZ - : 아래로

 권장 사용:
   CANRK 는 k_best(순수 중심 추정값)로 되돌리고, 조준 보정은 전부 AIM 으로 준다.
     can     CANRK=0.413  AIMX=0.025
     sponge  CANRK=0.040  AIMX=0.025  AIMY=-0.05
     bottle  CANRK=0.375  AIMX=0.025  AIMY=+0.04  AIMZ=+0.095

 사용: python3 patch_candetect_aim.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch_candetect_aim: AIM ---"

OLD = """    if verbose:
        print("CANDET area=%d n=%d  surf %s -> est %s"""
NEW = """    """ + MARK + """
    _aim = np.array([float(os.environ.get("AIMX", "0")),
                     float(os.environ.get("AIMY", "0")),
                     float(os.environ.get("AIMZ", "0"))])
    if np.any(_aim != 0):
        # 베이스 프레임 -> 월드.  Rb = diag(-1,-1,1)
        est = est + np.array([-_aim[0], -_aim[1], _aim[2]])
        if verbose:
            print("AIM base %s -> est %s" % (np.round(_aim, 4), np.round(est, 4)))
    if verbose:
        print("CANDET area=%d n=%d  surf %s -> est %s"""


def main():
    p = os.path.join(WORK, "can_detect.py")
    s = open(p).read()
    if MARK in s:
        print("[patch_candetect_aim] 이미 패치됨"); return
    n = s.count(OLD)
    if n != 1:
        raise SystemExit("앵커 %d 곳 (1이어야 함). 중단." % n)
    s = s.replace(OLD, NEW)
    shutil.copyfile(p, p + ".bak_aim")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("[patch_candetect_aim] ok  (백업: %s.bak_aim)" % p)


if __name__ == "__main__":
    main()
