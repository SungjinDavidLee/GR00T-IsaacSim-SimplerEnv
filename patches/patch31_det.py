#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch31_det.py -- run_groot31.py 에 ANCHOR=detect 를 붙인다.

  ANCHOR=follow  캔 좌표를 시뮬레이터 정답(get_privileged_state)에서 읽는다  ← 반칙
  ANCHOR=detect  캔 좌표를 wrist RGB-D 로 추정한다                          ← 새로 추가
  ANCHOR=fixed   기준점 고정 (위치 일반화 측정용). 그대로 둔다

 detect 모드에서도 privileged 좌표는 계속 읽지만 **평가·로깅에만** 쓴다.
 정책이 보는 프레임(OFF)과 z_floor 는 전부 추정치로 계산된다.

 RESULT 줄에 detx/dety/detz(추정 오차)가 추가된다.

 사용: python3 patch31_det.py [work_dir]
 원본은 .bak_det 로 백업. 이미 패치된 파일은 건너뛴다.
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch31_det: ANCHOR=detect ---"


def sub(s, old, new, what):
    n = s.count(old)
    if n != 1:
        raise SystemExit("[patch31_det] 실패: '%s' 앵커가 %d 곳 (1이어야 함). 중단." % (what, n))
    print("  ok:", what)
    return s.replace(old, new)


# ---------------------------------------------------------------- runner
R_OLD_IMP = '''import isaac_simpler_env_multi_object as base'''
R_NEW_IMP = MARK + '''
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import isaac_simpler_env_multi_object as base'''

R_OLD_ANCHOR = '''    elif ANCHOR.startswith("fix"):'''
R_NEW_ANCHOR = '''    elif ANCHOR.startswith("det"):
        # ''' + MARK + '''
        # 캔 위치를 wrist RGB-D 로 추정한다. privileged 좌표는 아래 오차 계산에만 쓴다.
        import can_detect
        anchor_w = can_detect.estimate_can_world(backend, verbose=True, truth=can_w)
        if anchor_w is None:
            print("ANCHOR=detect: 캔 검출 실패 -> CANREF/원배치로 폴백")
            anchor_w = (canref if canref is not None else CAN_NOMINAL).copy()
            det_err = np.array([9.99, 9.99, 9.99])
        else:
            det_err = anchor_w - can_w
        OFF = RbT.dot(anchor_w - bp) - LIB_OBJ - bias
        print("ANCHOR=detect  est", np.round(anchor_w, 4),
              " truth", np.round(can_w, 4),
              " err", np.round(det_err, 4))
    elif ANCHOR.startswith("fix"):'''

R_OLD_DET0 = '''    off_direct = env_vec("OFFREF")'''
R_NEW_DET0 = '''    det_err = np.array([0.0, 0.0, 0.0])   # ''' + MARK + '''
    off_direct = env_vec("OFFREF")'''

R_OLD_ZREF = '''    z_ref = float(anchor_w[2]) if (anchor_w is not None and ANCHOR.startswith("fix")) \\
        else float(can_w[2])'''
R_NEW_ZREF = '''    # ''' + MARK + '''  detect 모드도 추정 높이를 쓴다 (정답 높이 누설 차단)
    z_ref = float(anchor_w[2]) \\
        if (anchor_w is not None and ANCHOR[:3] in ("fix", "det")) \\
        else float(can_w[2])'''

R_OLD_RES = '''           "nclose=%d last_dy=%+.4f cammode=%s frontpos=%s") % ('''
R_NEW_RES = '''           "nclose=%d last_dy=%+.4f cammode=%s frontpos=%s "
           "detx=%+.4f dety=%+.4f detz=%+.4f anchor=%s") % ('''

R_OLD_RESV = '''        res["nclose"], res["last_dy"], CAMMODE, FRONTPOS))'''
R_NEW_RESV = '''        res["nclose"], res["last_dy"], CAMMODE, FRONTPOS,
        det_err[0], det_err[1], det_err[2], ANCHOR))'''


def patch_runner(p):
    s = open(p).read()
    if MARK in s:
        print("[patch31_det] run_groot31.py 이미 패치됨 — 건너뜀"); return False
    s = sub(s, R_OLD_IMP, R_NEW_IMP, "sys.path + import")
    s = sub(s, R_OLD_DET0, R_NEW_DET0, "det_err 초기화")
    s = sub(s, R_OLD_ANCHOR, R_NEW_ANCHOR, "ANCHOR=detect 분기")
    s = sub(s, R_OLD_ZREF, R_NEW_ZREF, "z_ref 에 detect 포함")
    s = sub(s, R_OLD_RES, R_NEW_RES, "RESULT 포맷")
    s = sub(s, R_OLD_RESV, R_NEW_RESV, "RESULT 값")
    shutil.copyfile(p, p + ".bak_det")
    open(p, "w").write(s)
    return True


# ---------------------------------------------------------------- pos_grid
G_OLD_HDR = '''                     "cammode", "frontpos", "log", "vid"])'''
G_NEW_HDR = '''                     "cammode", "frontpos", "log", "vid",
                     "det_dx", "det_dy", "det_dz", "anchor"])'''

G_OLD_ROW = '''                           r.get("cammode", ""), r.get("frontpos", ""), log, vid]'''
G_NEW_ROW = '''                           r.get("cammode", ""), r.get("frontpos", ""), log, vid,
                           r.get("detx", ""), r.get("dety", ""),
                           r.get("detz", ""), r.get("anchor", "")]  # ''' + MARK


def patch_grid(p):
    s = open(p).read()
    if MARK in s:
        print("[patch31_det] pos_grid.py 이미 패치됨 — 건너뜀"); return False
    if '"log", "vid"' not in s:
        raise SystemExit("[patch31_det] pos_grid.py 에 patch31 이 먼저 적용돼야 합니다.")
    s = sub(s, G_OLD_HDR, G_NEW_HDR, "CSV 헤더 det 열")
    s = sub(s, G_OLD_ROW, G_NEW_ROW, "CSV row det 값")
    shutil.copyfile(p, p + ".bak_det")
    open(p, "w").write(s)
    return True


def main():
    if not os.path.exists(os.path.join(WORK, "can_detect.py")):
        raise SystemExit("[patch31_det] can_detect.py 를 %s 에 먼저 두세요." % WORK)
    for name, fn in (("run_groot31.py", patch_runner), ("pos_grid.py", patch_grid)):
        p = os.path.join(WORK, name)
        print("[patch31_det]", p)
        if fn(p):
            py_compile.compile(p, doraise=True)
            print("  SYNTAX OK  (백업: %s.bak_det)" % p)
    print("[patch31_det] 완료")


if __name__ == "__main__":
    main()
