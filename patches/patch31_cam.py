#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch31_cam.py -- run_groot31.py 에 CAMSHIFT 를 추가한다.

 왜:
   측정 결과 XBIAS/YBIAS 를 25 mm 범위로 흔들어도 파지 잔차는 3.6 mm 밖에 안 움직였다.
   즉 정책은 state 프레임이 아니라 **wrist 영상**을 기준으로 손을 세운다.
   손과 카메라는 강체로 붙어 있으므로, 최종 손 자세는 **카메라 장착 위치**가 정한다.
   따라서 손가락이 캔을 물게 하려면 돌릴 손잡이는 bias 가 아니라 카메라 장착 오프셋이다.

 CAMSHIFT="dx,dy,dz"  panda_hand 프레임에서 D405Mount 를 이만큼 옮긴다 [m].
 MOUNTPRIM            기본 /World/Franka/panda_hand/D405Mount

 파일은 건드리지 않는다. 런타임 스테이지에서만 옮기므로 형 원본 USD/yaml 은 무손상.
 실물 Franka 에서는 이 값이 곧 **카메라 브래킷 치수**가 된다 — 그대로 이식된다.

 사용: python3 patch31_cam.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch31_cam: CAMSHIFT ---"

OLD = '''    bp, bq = robot.get_world_pose()'''
NEW = '''    ''' + MARK + '''
    _cs = env_vec("CAMSHIFT")
    if _cs is not None:
        try:
            from pxr import UsdGeom, Gf
            import omni.usd
            _mp = os.environ.get("MOUNTPRIM",
                                 "/World/Franka/panda_hand/D405Mount")
            _pr = omni.usd.get_context().get_stage().GetPrimAtPath(_mp)
            if not _pr or not _pr.IsValid():
                raise RuntimeError("mount prim 없음: " + _mp)
            _xf = UsdGeom.Xformable(_pr)
            _op = None
            for _o in _xf.GetOrderedXformOps():
                if _o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    _op = _o
                    break
            if _op is None:
                _op = _xf.AddTranslateOp()
                _old = Gf.Vec3d(0.0, 0.0, 0.0)
            else:
                _old = _op.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
            _new = Gf.Vec3d(float(_old[0]) + float(_cs[0]),
                            float(_old[1]) + float(_cs[1]),
                            float(_old[2]) + float(_cs[2]))
            _op.Set(_new)
            print("CAMSHIFT %s  mount translate [%.4f %.4f %.4f] -> [%.4f %.4f %.4f]"
                  % (np.round(_cs, 4), _old[0], _old[1], _old[2],
                     _new[0], _new[1], _new[2]))
        except Exception as _e:
            print("CAMSHIFT 적용 실패:", _e)
    else:
        print("CAMSHIFT none (카메라 장착 원위치)")

    bp, bq = robot.get_world_pose()'''

FIT_OLD = """                    fit = abs(_e2[1]) < 0.0063"""
FIT_NEW = """                    fit = (abs(_e2[1]) < FITDY and abs(_e2[0]) < FITDX
                           and abs(_e2[2]) < FITDZ)"""

FITDEF_OLD = """    res = dict(ok=0, close_step=-1, dx=9.99, dy=9.99, dz=9.99, fit=0,"""
FITDEF_NEW = """    # """ + MARK + """  FIT 은 dy 만 보고 있었다. dx 가 34 mm 어긋나도 FIT=1 이 찍혔다.
    FITDY = float(os.environ.get("FITDY", "0.0063"))
    FITDX = float(os.environ.get("FITDX", "0.015"))
    FITDZ = float(os.environ.get("FITDZ", "0.020"))
    print("FIT 기준  |dx|<%.4f  |dy|<%.4f  |dz|<%.4f" % (FITDX, FITDY, FITDZ))
    res = dict(ok=0, close_step=-1, dx=9.99, dy=9.99, dz=9.99, fit=0,"""

RES_OLD = '''           "detx=%+.4f dety=%+.4f detz=%+.4f anchor=%s") % ('''
RES_NEW = '''           "detx=%+.4f dety=%+.4f detz=%+.4f anchor=%s camshift=%s") % ('''
RESV_OLD = '''        det_err[0], det_err[1], det_err[2], ANCHOR))'''
RESV_NEW = '''        det_err[0], det_err[1], det_err[2], ANCHOR,
        os.environ.get("CAMSHIFT", "0,0,0").replace(" ", "")))'''


def sub(s, old, new, what):
    n = s.count(old)
    if n != 1:
        raise SystemExit("[patch31_cam] '%s' 앵커 %d 곳 (1이어야 함). 중단." % (what, n))
    print("  ok:", what)
    return s.replace(old, new)


def main():
    p = os.path.join(WORK, "run_groot31.py")
    s = open(p).read()
    if MARK in s:
        print("[patch31_cam] 이미 패치됨 — 건너뜀"); return
    if "patch31_det" not in s:
        raise SystemExit("[patch31_cam] patch31_det.py 를 먼저 적용하세요.")
    s = sub(s, OLD, NEW, "CAMSHIFT 적용 블록")
    s = sub(s, FITDEF_OLD, FITDEF_NEW, "FIT 임계 정의")
    s = sub(s, FIT_OLD, FIT_NEW, "FIT 판정에 dx/dz 포함")
    s = sub(s, RES_OLD, RES_NEW, "RESULT 포맷")
    s = sub(s, RESV_OLD, RESV_NEW, "RESULT 값")
    shutil.copyfile(p, p + ".bak_cam")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("  SYNTAX OK  (백업: %s.bak_cam)" % p)


if __name__ == "__main__":
    main()
