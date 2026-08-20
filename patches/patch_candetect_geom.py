#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_candetect_geom.py -- can_detect.py 에 DETGEOM=top 을 추가한다.
                           **물체별 상수(CANRAD/CANRK/CANZOFF)를 없앤다.**

 지금까지의 문제:
   xy : 보이는 표면 중심 + CANRAD*CANRK 로 중심축을 추정 -> 물체마다 반지름을 알아야 함
   z  : CANZOFF 상수 -> 물체마다 정답과 1회 교정해야 함  (= 특권 정보 사용)

 DETGEOM=top 이 쓰는 것 (전부 depth 에서 나온다):
   1. 마스크 화소를 월드 점구름으로 역투영
   2. **윗면 띠**(최고점에서 TOPBAND 이내)의 중앙값 -> 물체의 수직축 xy
      원기둥이든 직육면체든 위에서 보면 윗면 중심이 곧 축이다. 반지름 불필요.
   3. 마스크 바깥 고리 영역의 depth 중앙값 -> **테이블 높이**
   4. 물체 높이 = 윗면 - 테이블,  파지 높이 = 윗면 - GRIPFRAC * 높이
      GRIPFRAC 는 물체가 아니라 **파지 방식**의 상수다 (캔 0.44 / 스펀지 0.50 실측 -> 0.47)

 env:
   DETGEOM=top       켜기 (기본 ray = 기존 반지름 보정 방식)
   TOPBAND=0.010     윗면으로 볼 두께 [m]
   GRIPFRAC=0.47     윗면에서 아래로 높이의 이 비율 지점을 잡는다
   RINGPX=60         테이블 높이 추정용 고리 두께 [px]

 사용: python3 patch_candetect_geom.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch_candetect_geom: DETGEOM=top ---"

OLD = """    surf = P_w.mean(axis=0)"""
NEW = """    """ + MARK + """
    if os.environ.get("DETGEOM", "ray").startswith("top"):
        TOPBAND = float(os.environ.get("TOPBAND", "0.010"))
        GRIPFRAC = float(os.environ.get("GRIPFRAC", "0.47"))
        RINGPX = int(os.environ.get("RINGPX", "60"))
        # (2) 윗면 띠의 중앙값 = 물체의 수직축 xy. 반지름 상수 불필요.
        ztop = float(np.percentile(P_w[:, 2], 97))
        topsel = P_w[P_w[:, 2] > ztop - TOPBAND]
        if len(topsel) < 15:
            topsel = P_w[np.argsort(P_w[:, 2])[-max(15, len(P_w) // 20):]]
        cxy = np.median(topsel[:, :2], axis=0)
        # (3) 마스크 바깥 고리에서 테이블 높이
        k = np.ones((RINGPX, RINGPX), np.uint8)
        ring = (cv2.dilate(mask, k) > 0) & (mask == 0)
        ry, rx = np.nonzero(ring)
        rz = dep[ry, rx]
        okr = (rz > 0.02) & (rz < 3.0)
        ztab = None
        if okr.sum() > 50:
            rx, ry, rz = rx[okr], ry[okr], rz[okr]
            Rp = np.stack([(rx - cx) / fx * rz, (ry - cy) / fy * rz, rz], axis=1)
            Rw = cp[None, :] + (R.dot(RC.dot(Rp.T))).T
            ztab = float(np.median(Rw[:, 2]))
        if ztab is None or not (0.0 < ztop - ztab < 0.6):
            ztab = ztop - 0.05
            print("[can_detect] 테이블 높이 추정 실패 -> 높이 0.05 m 로 가정")
        hgt = ztop - ztab
        est = np.array([cxy[0], cxy[1], ztop - GRIPFRAC * hgt])
        if verbose:
            print("CANDET(top) area=%d n=%d  ztop %.4f ztable %.4f 높이 %.4f "
                  "-> est %s  (gripfrac=%.2f, 물체상수 없음)"
                  % (area, len(zs), ztop, ztab, hgt, np.round(est, 4), GRIPFRAC))
            if truth is not None:
                e = est - np.asarray(truth).reshape(3)
                print("CANDET(top) err dx %+.4f dy %+.4f dz %+.4f | xy %.4f m"
                      % (e[0], e[1], e[2], float(np.linalg.norm(e[:2]))))
                t = np.asarray(truth).reshape(3)
                if hgt > 1e-6:
                    print("CANDET(top) calib  gripfrac_best=%.3f" % ((ztop - t[2]) / hgt))
        if DUMP:
            os.makedirs(DUMP, exist_ok=True)
            ov = rgb.copy()
            ov[mask > 0] = (0.35 * ov[mask > 0] + 0.65 * np.array([0, 255, 0])).astype(np.uint8)
            cv2.imwrite(os.path.join(DUMP, "det.png"), cv2.cvtColor(ov, cv2.COLOR_RGB2BGR))
        return est

    surf = P_w.mean(axis=0)"""


def main():
    p = os.path.join(WORK, "can_detect.py")
    s = open(p).read()
    if MARK in s:
        print("[patch_candetect_geom] 이미 패치됨"); return
    n = s.count(OLD)
    if n != 1:
        raise SystemExit("앵커 %d 곳 (1이어야 함). 중단." % n)
    s = s.replace(OLD, NEW)
    shutil.copyfile(p, p + ".bak_geom")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("[patch_candetect_geom] ok  (백업: %s.bak_geom)" % p)


if __name__ == "__main__":
    main()
