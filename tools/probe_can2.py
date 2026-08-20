#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_can2.py -- probe_can 의 후속. 오차 112 mm 의 원인을 세 갈래로 동시에 가른다.

  Q1  'wrist' 키가 정말 손에 달린 D405 인가?  ('up' 과 배열이 동일한지 직접 비교)
  Q2  intrinsics 공칭 87deg 가 틀렸는가?      (USD Camera prim 의 focalLength /
                                              horizontalAperture 로 정확히 계산)
  Q3  축 규약이 틀렸는가?                     (4가지 규약 전수 + 정답으로 역산한
                                              implied fx/fy 출력)

 팔은 움직이지 않는다. 홈 자세 한 프레임만 본다. GR00T 서버는 켜져 있어도 무방하다.

 실행:
   cd /home/data/groot/MIGRATE/ENV
   python3 /home/data/groot/work/probe_can2.py --config config/environment_groot.yaml
"""
import os, sys
if int(os.environ.get("NOWIN", "1")):
    sys.argv += ["--no-window"]
import numpy as np
import cv2
import isaac_simpler_env_multi_object as base

DUMP = os.environ.get("DUMP", "/home/data/groot/work/probe")
SMIN = int(os.environ.get("SMIN", "90"))
VMIN = int(os.environ.get("VMIN", "50"))
KEY = os.environ.get("CAMKEY", "wrist")


def red_mask(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m = (cv2.inRange(hsv, (0, SMIN, VMIN), (10, 255, 255)) |
         cv2.inRange(hsv, (170, SMIN, VMIN), (179, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None, None, 0, None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    bb = (st[i, cv2.CC_STAT_LEFT], st[i, cv2.CC_STAT_TOP],
          st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT])
    return (lab == i).astype(np.uint8), ce[i], int(st[i, cv2.CC_STAT_AREA]), bb


def dump_cameras():
    """스테이지의 모든 Camera prim 을 열거하고 내부 파라미터를 찍는다."""
    out = []
    try:
        from pxr import UsdGeom, Usd
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Camera":
                continue
            path = str(prim.GetPath())
            g = lambda n: (prim.GetAttribute(n).Get()
                           if prim.GetAttribute(n).IsValid() else None)
            M = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())).T
            out.append(dict(path=path, fl=g("focalLength"),
                            ha=g("horizontalAperture"), va=g("verticalAperture"),
                            hao=g("horizontalApertureOffset"),
                            vao=g("verticalApertureOffset"),
                            pos=M[:3, 3].copy(), R=M[:3, :3].copy()))
    except Exception as e:
        print("  [USD 열거 실패]", e)
    return out


def main():
    os.makedirs(DUMP, exist_ok=True)
    cfg = base.CONFIG
    backend = base.IsaacBackend(cfg)
    task = base.LiftObjectTask(cfg)
    env = base.FrankaSimplerEnv(cfg, backend, task)
    env.reset()
    robot, _ = backend._require_ready()
    for _ in range(60):
        backend.advance_control_step(render=True)

    imgs = backend.get_camera_images()

    # ---------------- Q1. wrist 가 up 과 같은 스트림인가 ----------------
    print("=" * 68)
    print("Q1. 스트림 동일성 검사")
    ks = sorted([str(k) for k in imgs])
    for i in range(len(ks)):
        for j in range(i + 1, len(ks)):
            a, b = np.asarray(imgs[ks[i]]), np.asarray(imgs[ks[j]])
            if a.shape == b.shape and np.array_equal(a, b):
                print("  !! '%s' 와 '%s' 가 **완전히 동일한 배열**" % (ks[i], ks[j]))
    print("  (위에 아무것도 안 찍히면 모든 스트림이 서로 다르다 = 정상)")

    # ---------------- Q2. USD Camera prim 의 실제 intrinsics ----------------
    print("=" * 68)
    print("Q2. 스테이지의 Camera prim 과 내부 파라미터")
    cams = dump_cameras()
    for c in cams:
        line = "  %s" % c["path"]
        if c["fl"] and c["ha"]:
            hf = 2 * np.degrees(np.arctan(c["ha"] / (2.0 * c["fl"])))
            vf = (2 * np.degrees(np.arctan(c["va"] / (2.0 * c["fl"])))
                  if c["va"] else float("nan"))
            line += "\n     focalLength=%.4f hAperture=%.4f vAperture=%s" % (
                c["fl"], c["ha"], c["va"])
            line += "\n     -> hFOV=%.2f deg  vFOV=%.2f deg" % (hf, vf)
        line += "\n     worldPos=%s" % np.round(c["pos"], 4)
        print(line)
    if not cams:
        print("  Camera prim 을 못 찾음")

    # ---------------- 대상 카메라 ----------------
    if KEY not in imgs or (KEY + "_depth") not in imgs:
        print("!! 키 없음:", KEY); return 1
    rgb = np.asarray(imgs[KEY])[:, :, :3]
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    dep = np.asarray(imgs[KEY + "_depth"]).astype(np.float64)
    if dep.ndim == 3:
        dep = dep[:, :, 0]
    dep[~np.isfinite(dep)] = 0.0
    h, w = rgb.shape[:2]

    mask, cent, area, bb = red_mask(rgb)
    cv2.imwrite(os.path.join(DUMP, "c2_rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if mask is None:
        print("!! 마스크 실패"); return 2
    ov = rgb.copy()
    ov[mask > 0] = (0.35 * ov[mask > 0] + 0.65 * np.array([0, 255, 0])).astype(np.uint8)
    cv2.rectangle(ov, (bb[0], bb[1]), (bb[0] + bb[2], bb[1] + bb[3]), (255, 255, 0), 2)
    cv2.circle(ov, (int(cent[0]), int(cent[1])), 6, (255, 0, 255), 2)
    cv2.imwrite(os.path.join(DUMP, "c2_mask.png"), cv2.cvtColor(ov, cv2.COLOR_RGB2BGR))
    print("=" * 68)
    print("마스크 area=%d px  bbox=(x%d y%d w%d h%d)  centroid=(%.1f, %.1f)  image %dx%d"
          % (area, bb[0], bb[1], bb[2], bb[3], cent[0], cent[1], w, h))
    edge = (bb[0] <= 1 or bb[1] <= 1 or bb[0] + bb[2] >= w - 1 or bb[1] + bb[3] >= h - 1)
    print("  화면 경계에 물림: %s  %s" % (edge, "→ centroid 가 캔 중심이 아니다" if edge else ""))

    dm = dep[mask > 0]
    dm = dm[(dm > 0.02) & (dm < 3.0)]
    z = float(np.median(dm))
    print("  캔 표면 z(median)=%.4f  p10=%.4f p90=%.4f  n=%d"
          % (z, np.percentile(dm, 10), np.percentile(dm, 90), dm.size))

    # ---------------- Q3. 축 규약 + implied focal ----------------
    truth = None
    try:
        for k, v in backend.get_privileged_state().items():
            if "can" in str(k).lower():
                vv = np.asarray(v, dtype=np.float64).reshape(-1)
                if vv.size >= 3:
                    truth = vv[:3]; break
    except Exception:
        pass
    if truth is None:
        print("정답 캔 위치 없음"); return 3

    # 이 키에 해당하는 카메라 prim 을 고른다: 손(panda_hand) 아래 것 우선
    pick = None
    for c in cams:
        if "panda_hand" in c["path"] or "D405" in c["path"]:
            pick = c; break
    if pick is None and cams:
        pick = cams[0]
    if pick is None:
        print("Camera prim 없음"); return 4
    print("=" * 68)
    print("Q3. 사용한 Camera prim:", pick["path"])
    R = pick["R"].copy()
    sc = np.linalg.norm(R, axis=0)
    print("  R 열 노름(스케일) %s  det=%.4f" % (np.round(sc, 4), np.linalg.det(R)))
    R = R / np.maximum(sc, 1e-12)
    cp = pick["pos"]
    print("  R(정규화)\n", np.round(R, 4))

    v_world = truth - cp
    print("  cam->can (월드) %s   |거리| %.4f" % (np.round(v_world, 4),
                                                 float(np.linalg.norm(v_world))))
    v_local = R.T.dot(v_world)
    print("  cam->can (prim 로컬) %s" % np.round(v_local, 4))

    CONV = {
        "OpenCV (+X right,+Y down,+Z fwd)": np.eye(3),
        "USD    (+X right,+Y up,  -Z fwd)": np.diag([1.0, -1.0, -1.0]),
        "flipX  (-X,+Y,-Z)": np.diag([-1.0, 1.0, -1.0]),
        "flipXY (-X,-Y,+Z)": np.diag([-1.0, -1.0, 1.0]),
    }
    u, v = float(cent[0]), float(cent[1])
    cx, cy = (w - 1) * 0.5, (h - 1) * 0.5
    print("\n  규약별: 정답을 맞추려면 어떤 intrinsics 여야 하는가")
    print("  %-34s %8s %8s %9s" % ("규약", "impl_fx", "impl_fy", "Z_needed"))
    for name, Rc in CONV.items():
        vc = Rc.dot(v_local)          # 카메라 광학 프레임에서 본 캔
        if abs(vc[0]) < 1e-9 or abs(vc[1]) < 1e-9:
            continue
        ifx = (u - cx) * vc[2] / vc[0]
        ify = (v - cy) * vc[2] / vc[1]
        print("  %-34s %8.1f %8.1f %9.4f" % (name, ifx, ify, vc[2]))
    print("\n  측정 depth z = %.4f  <-- Z_needed 와 가까운 규약이 정답이다." % z)
    print("  그 규약의 impl_fx / impl_fy 가 실제 intrinsics 다.")
    if pick["fl"] and pick["ha"]:
        fx_usd = pick["fl"] / pick["ha"] * w
        fy_usd = (pick["fl"] / pick["va"] * h) if pick["va"] else fx_usd
        print("  USD 속성 기반 fx=%.1f fy=%.1f  (공칭87deg 기반은 %.1f)"
              % (fx_usd, fy_usd, (w * 0.5) / np.tan(np.deg2rad(87.0) * 0.5)))
    print("\n정답 캔 월드 %s" % np.round(truth, 4))
    print("진단 이미지:", DUMP)
    return 0


if __name__ == "__main__":
    sys.exit(main())
