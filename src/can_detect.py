#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_detect.py -- wrist D405 의 RGB + depth 만으로 캔의 월드 좌표를 추정한다.

 시뮬레이터의 정답 좌표(get_privileged_state)를 쓰지 않는다.
 실물 이식 시 바뀌는 것은 두 가지뿐이다:
   · intrinsics  -> librealsense 의 rs2_intrinsics 에서 읽는다
   · 카메라 pose -> USD 조회 대신 FK(손 링크) + 손-카메라 고정 변환

 ── 측정으로 확정된 값 (2026-08-18, probe_can2) ────────────────────────
   카메라 prim   /World/Franka/panda_hand/D405Mount/Camera_D405   (손에 부착)
   해상도        1280x720
   focalLength   1.93   hAperture 5.2644   vAperture 2.9612
   -> fx = fy = 469.27      (공칭 87deg 가정값 674.4 는 틀렸다)
   축 규약       USD (+X right, +Y up, -Z forward)
   depth         distance_to_image_plane, 미터 단위
   ※ 'up' 과 'wrist' 는 같은 카메라의 같은 스트림이다 (배열 동일 확인).

 ── env ───────────────────────────────────────────────────────────────
   CANRAD   0.0329  캔 반지름 [m]. 씬 로그 [Object bounds] half_extents_xy
   CANRK    1.0     보이는 표면 -> 중심축 보정 계수 (반지름의 몇 배 밀 것인가)
   CANZOFF  0.0     z 상수 보정 [m]. 빨강 영역 3D 중심 z 와 물체 원점 z 의 차
   CAMKEY   wrist
   CAMPRIM  /World/Franka/panda_hand/D405Mount/Camera_D405
   SMIN 90 / VMIN 50 / HUE red      HSV 마스크 임계
   CANDETWARM 5    검출 전 렌더 스텝 수
   CANDETDUMP ""   비우면 저장 안 함. 경로를 주면 진단 PNG 저장
"""
import os
import numpy as np
import cv2

CANRAD = float(os.environ.get("CANRAD", "0.0329"))
CANRK = float(os.environ.get("CANRK", "1.0"))
CANZOFF = float(os.environ.get("CANZOFF", "0.0"))
CAMKEY = os.environ.get("CAMKEY", "wrist")
CAMPRIM = os.environ.get("CAMPRIM",
                         "/World/Franka/panda_hand/D405Mount/Camera_D405")
SMIN = int(os.environ.get("SMIN", "90"))
VMIN = int(os.environ.get("VMIN", "50"))
HUE = os.environ.get("HUE", "red")
WARM = int(os.environ.get("CANDETWARM", "5"))
DUMP = os.environ.get("CANDETDUMP", "")

# USD 광학 규약: prim 로컬축 -> 카메라 광학축(+X right, +Y down, +Z fwd)
RC = np.diag([1.0, -1.0, -1.0])

_intr = {}


def _cam_prim():
    from pxr import UsdGeom, Usd
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    p = stage.GetPrimAtPath(CAMPRIM)
    if not p or not p.IsValid():
        for q in stage.Traverse():
            if q.GetTypeName() == "Camera" and "panda_hand" in str(q.GetPath()):
                p = q
                break
    return p, UsdGeom, Usd


def cam_pose_world():
    """카메라 prim 의 현재 월드 pose. 손과 함께 움직이므로 매번 다시 읽는다."""
    p, UsdGeom, Usd = _cam_prim()
    if not p or not p.IsValid():
        return None, None
    M = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())).T
    R = M[:3, :3].copy()
    R = R / np.maximum(np.linalg.norm(R, axis=0), 1e-12)
    return M[:3, 3].copy(), R


def intrinsics(w, h):
    key = (w, h)
    if key in _intr:
        return _intr[key]
    fx = fy = None
    try:
        p, _, _ = _cam_prim()
        fl = p.GetAttribute("focalLength").Get()
        ha = p.GetAttribute("horizontalAperture").Get()
        va = p.GetAttribute("verticalAperture").Get()
        fx = fl / ha * w
        fy = fl / va * h
    except Exception as e:
        print("[can_detect] intrinsics USD 조회 실패:", e)
    if fx is None:
        fx = fy = 469.27      # probe_can2 로 확정한 값
        print("[can_detect] USD 조회 실패 -> 실측 확정값 fx=fy=469.27 사용")
    _intr[key] = (float(fx), float(fy), (w - 1) * 0.5, (h - 1) * 0.5)
    return _intr[key]


def red_mask(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    if HUE == "red":
        m = (cv2.inRange(hsv, (0, SMIN, VMIN), (10, 255, 255)) |
             cv2.inRange(hsv, (170, SMIN, VMIN), (179, 255, 255)))
    else:
        a, b, c, d = [int(v) for v in HUE.split(",")]
        m = (cv2.inRange(hsv, (a, SMIN, VMIN), (b, 255, 255)) |
             cv2.inRange(hsv, (c, SMIN, VMIN), (d, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None, 0
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    return (lab == i).astype(np.uint8), int(st[i, cv2.CC_STAT_AREA])


def estimate_can_world(backend, verbose=True, truth=None):
    """캔의 월드 좌표 추정치를 돌려준다. 실패하면 None."""
    for _ in range(WARM):
        try:
            backend.advance_control_step(render=True)
        except Exception:
            break
    imgs = backend.get_camera_images()
    if CAMKEY not in imgs or (CAMKEY + "_depth") not in imgs:
        print("[can_detect] 카메라 키 없음:", CAMKEY, sorted(map(str, imgs)))
        return None
    rgb = np.asarray(imgs[CAMKEY])[:, :, :3]
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    dep = np.asarray(imgs[CAMKEY + "_depth"]).astype(np.float64)
    if dep.ndim == 3:
        dep = dep[:, :, 0]
    dep[~np.isfinite(dep)] = 0.0
    h, w = rgb.shape[:2]
    if dep.shape[:2] != (h, w):
        dep = cv2.resize(dep, (w, h), interpolation=cv2.INTER_NEAREST)

    mask, area = red_mask(rgb)
    if mask is None or area < 40:
        print("[can_detect] 마스크 실패 area=%d" % area)
        return None

    fx, fy, cx, cy = intrinsics(w, h)
    cp, R = cam_pose_world()
    if cp is None:
        print("[can_detect] 카메라 pose 조회 실패")
        return None

    ys, xs = np.nonzero(mask)
    zs = dep[ys, xs]
    ok = (zs > 0.02) & (zs < 3.0)
    # depth 이상치 제거 (MAD)
    if ok.sum() > 30:
        z_ok = zs[ok]
        med = np.median(z_ok)
        mad = np.median(np.abs(z_ok - med)) + 1e-9
        ok = ok & (np.abs(zs - med) < 6.0 * mad)
    if ok.sum() < 30:
        print("[can_detect] 유효 depth 화소 부족 n=%d" % int(ok.sum()))
        return None
    xs, ys, zs = xs[ok], ys[ok], zs[ok]

    # 화소 -> 카메라 광학 프레임 -> 월드
    Xc = (xs - cx) / fx * zs
    Yc = (ys - cy) / fy * zs
    P_opt = np.stack([Xc, Yc, zs], axis=1)          # (N,3)
    P_w = cp[None, :] + (R.dot(RC.dot(P_opt.T))).T  # (N,3)

    surf = P_w.mean(axis=0)
    # 보이는 것은 캔의 '앞면'이다. 수평 시선 방향으로 반지름만큼 밀어 중심축을 만든다.
    dvec = surf - cp
    dh = np.array([dvec[0], dvec[1], 0.0])
    nh = float(np.linalg.norm(dh))
    if nh < 1e-6:
        print("[can_detect] 수평 시선 성분이 0 — 보정 불가")
        return None
    est = surf + (CANRAD * CANRK) * (dh / nh)
    est[2] = surf[2] + CANZOFF

    if verbose:
        print("CANDET area=%d n=%d  surf %s -> est %s  (fx=%.1f rad=%.4f k=%.2f zoff=%+.4f)"
              % (area, len(zs), np.round(surf, 4), np.round(est, 4),
                 fx, CANRAD, CANRK, CANZOFF))
        if truth is not None:
            e = est - np.asarray(truth).reshape(3)
            print("CANDET err dx %+.4f dy %+.4f dz %+.4f | xy %.4f m  (허용 dy 0.0063)"
                  % (e[0], e[1], e[2], float(np.linalg.norm(e[:2]))))
            # 이번 프레임에서 최적이었을 보정 계수 — 교정용 참고값
            t = np.asarray(truth).reshape(3)
            k_best = float((t[:2] - surf[:2]).dot(dh[:2] / nh) / max(CANRAD, 1e-9))
            print("CANDET calib  k_best=%.3f  zoff_best=%+.4f" % (k_best, t[2] - surf[2]))
    if DUMP:
        os.makedirs(DUMP, exist_ok=True)
        ov = rgb.copy()
        ov[mask > 0] = (0.35 * ov[mask > 0] + 0.65 * np.array([0, 255, 0])).astype(np.uint8)
        cv2.imwrite(os.path.join(DUMP, "det.png"), cv2.cvtColor(ov, cv2.COLOR_RGB2BGR))
    return est
