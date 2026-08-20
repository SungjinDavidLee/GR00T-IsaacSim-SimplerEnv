#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_can.py -- wrist RGB + wrist_depth 로 캔 위치를 추정하고, 시뮬레이터 정답과 비교한다.

 목적: run_groot31.py 를 고치기 **전에** "depth 로 캔을 ±6.3 mm 안에 찾을 수 있는가"만
       따로 측정한다. 이게 안 되면 통합해봐야 의미가 없다.

 GR00T 서버 불필요. 정책을 호출하지 않는다. 홈잉만 하고 한 장 찍어서 추정한다.

 실행:
   cd /home/data/groot/MIGRATE/ENV
   source /home/data/groot/venv-isaacsim/bin/activate
   export NOWIN=1 OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/data/groot/MIGRATE/ENV
   python3 /home/data/groot/work/probe_can.py --config config/environment_groot.yaml

 env:
   CANRAD=0.033   캔 반지름 [m]. 옆면 표면점 → 중심축 보정에 쓴다
   HUE=red        red | 직접 지정 "hlo1,hhi1,hlo2,hhi2"
   SMIN=90 VMIN=50
   DUMP=/home/data/groot/work/probe   진단 이미지 저장 위치
"""
import os, sys, json
if int(os.environ.get("NOWIN", "1")):
    sys.argv += ["--no-window"]
import numpy as np
import cv2
import isaac_simpler_env_multi_object as base

DUMP = os.environ.get("DUMP", "/home/data/groot/work/probe")
CANRAD = float(os.environ.get("CANRAD", "0.033"))
SMIN = int(os.environ.get("SMIN", "90"))
VMIN = int(os.environ.get("VMIN", "50"))


def describe(name, a):
    a = np.asarray(a)
    print("  %-14s shape=%s dtype=%s min=%.4f max=%.4f median=%.4f" % (
        name, a.shape, a.dtype, float(np.nanmin(a)), float(np.nanmax(a)),
        float(np.nanmedian(a))))


def to_depth_m(d):
    """depth 배열을 미터 단위 2D 로 정규화. 단위 추정 결과를 함께 반환."""
    d = np.asarray(d)
    if d.ndim == 3:
        d = d[:, :, 0]
    d = d.astype(np.float64)
    d[~np.isfinite(d)] = 0.0
    med = float(np.median(d[d > 0])) if (d > 0).any() else 0.0
    unit = "m"
    if med > 20.0:          # 밀리미터로 들어오는 경우
        d = d / 1000.0
        unit = "mm->m"
    return d, unit, med


def intrinsics(shape, cfg):
    """fx, fy, cx, cy 를 구한다. 출처를 함께 반환한다."""
    h, w = shape[:2]
    cx, cy = (w - 1) * 0.5, (h - 1) * 0.5
    src = None
    fov_deg = None
    try:
        wc = cfg["cameras"]["wrist"]
        for k in ("fov_deg", "horizontal_fov_deg", "hfov_deg", "fovx", "fov"):
            if k in wc:
                fov_deg = float(wc[k]); src = "yaml:" + k; break
        if fov_deg is None:
            for k in ("fovy", "vertical_fov_deg", "vfov_deg"):
                if k in wc:
                    fov_deg = float(wc[k]); src = "yaml:" + k
                    f = (h * 0.5) / np.tan(np.deg2rad(fov_deg) * 0.5)
                    return f, f, cx, cy, src
    except Exception:
        pass
    if fov_deg is None:
        fov_deg = 87.0      # RealSense D405 RGB 수평 FOV 공칭값
        src = "D405 공칭 87deg [확인 필요]"
    f = (w * 0.5) / np.tan(np.deg2rad(fov_deg) * 0.5)
    return f, f, cx, cy, src


def red_mask(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    spec = os.environ.get("HUE", "red")
    if spec == "red":
        m = (cv2.inRange(hsv, (0, SMIN, VMIN), (10, 255, 255)) |
             cv2.inRange(hsv, (170, SMIN, VMIN), (179, 255, 255)))
    else:
        a, b, c, d = [int(v) for v in spec.split(",")]
        m = (cv2.inRange(hsv, (a, SMIN, VMIN), (b, 255, 255)) |
             cv2.inRange(hsv, (c, SMIN, VMIN), (d, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None, None, 0
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (lab == i).astype(np.uint8), cent[i], int(stats[i, cv2.CC_STAT_AREA])


def cam_pose_world(backend):
    """wrist 카메라 prim 의 월드 pose 를 여러 경로로 시도한다."""
    tries = []
    for attr in ("cameras", "_cameras", "sensors", "_sensors"):
        obj = getattr(backend, attr, None)
        if isinstance(obj, dict):
            for k in obj:
                if "wrist" in str(k).lower():
                    tries.append(("%s[%s]" % (attr, k), obj[k]))
    for name, c in tries:
        for meth in ("get_world_pose", "get_world_poses"):
            f = getattr(c, meth, None)
            if f is None:
                continue
            try:
                p, q = f()
                p = np.asarray(p, dtype=np.float64).reshape(-1)[:3]
                q = np.asarray(q, dtype=np.float64).reshape(-1)[:4]
                return p, q, name + "." + meth
            except Exception:
                pass
    try:
        from pxr import UsdGeom, Usd
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            s = str(prim.GetPath())
            if "wrist" in s.lower() or "D405" in s:
                M = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                M = np.array(M).T
                return M[:3, 3].copy(), M[:3, :3].copy(), "USD:" + s
    except Exception as e:
        print("  USD 경로 실패:", e)
    return None, None, None


def quat_to_R(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float64)


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
    keys = sorted([str(k) for k in imgs])
    print("CAMERA KEYS", keys)
    print("-- 스트림 실측 --")
    for k in keys:
        describe(k, imgs[k])

    wk = "wrist" if "wrist" in imgs else None
    dk = "wrist_depth" if "wrist_depth" in imgs else None
    if wk is None or dk is None:
        print("!! wrist / wrist_depth 키를 못 찾음. 중단"); return 1

    rgb = np.asarray(imgs[wk])[:, :, :3]
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    dep, unit, med = to_depth_m(imgs[dk])
    print("depth 단위 판정: %s (median %.4f -> %.4f m)" % (unit, med, np.median(dep[dep > 0])))
    if rgb.shape[:2] != dep.shape[:2]:
        print("!! RGB %s 와 depth %s 해상도 불일치 — depth 를 RGB 에 맞춰 리사이즈"
              % (rgb.shape[:2], dep.shape[:2]))
        dep = cv2.resize(dep, (rgb.shape[1], rgb.shape[0]),
                         interpolation=cv2.INTER_NEAREST)

    fx, fy, cx, cy, isrc = intrinsics(rgb.shape, cfg)
    print("intrinsics fx=%.2f fy=%.2f cx=%.1f cy=%.1f  (%s)" % (fx, fy, cx, cy, isrc))

    mask, cent, area = red_mask(rgb)
    cv2.imwrite(os.path.join(DUMP, "wrist_rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    dv = dep.copy(); dv[dv <= 0] = np.nan
    lo, hi = np.nanpercentile(dv, 2), np.nanpercentile(dv, 98)
    vis = np.clip((dep - lo) / max(hi - lo, 1e-6), 0, 1)
    cv2.imwrite(os.path.join(DUMP, "wrist_depth.png"),
                cv2.applyColorMap((vis * 255).astype(np.uint8), cv2.COLORMAP_TURBO))
    if mask is None:
        print("!! 빨강 마스크에서 캔을 못 찾음. area=0")
        print("   -> probe/wrist_rgb.png 를 보고 HUE/SMIN/VMIN 을 조정하거나,")
        print("      홈 자세에서 캔이 아예 화면 밖일 가능성.")
        return 2
    print("mask area=%d px  centroid=(%.1f, %.1f)" % (area, cent[0], cent[1]))
    ov = rgb.copy(); ov[mask > 0] = (0.4 * ov[mask > 0] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)
    cv2.circle(ov, (int(cent[0]), int(cent[1])), 5, (255, 255, 0), 2)
    cv2.imwrite(os.path.join(DUMP, "wrist_mask.png"), cv2.cvtColor(ov, cv2.COLOR_RGB2BGR))

    dm = dep[mask > 0]
    dm = dm[(dm > 0.02) & (dm < 3.0)]
    if dm.size < 20:
        print("!! 마스크 영역의 유효 depth 화소가 %d개뿐. 중단" % dm.size); return 3
    z = float(np.median(dm))
    print("캔 표면 depth  median=%.4f m  p10=%.4f p90=%.4f  (n=%d)"
          % (z, np.percentile(dm, 10), np.percentile(dm, 90), dm.size))

    u, v = float(cent[0]), float(cent[1])
    p_cam = np.array([(u - cx) / fx * z, (v - cy) / fy * z, z])   # OpenCV 규약 가정
    print("캔 (카메라 프레임, OpenCV 규약 가정)", np.round(p_cam, 4))
    print("  ※ Isaac Sim 카메라 축 규약이 다르면 부호가 바뀐다. 아래 오차로 판정할 것.")

    cp, cr, csrc = cam_pose_world(backend)
    print("카메라 월드 pose 출처:", csrc)
    est_w = None
    if cp is not None:
        R = cr if (isinstance(cr, np.ndarray) and cr.shape == (3, 3)) else quat_to_R(cr)
        print("  cam pos", np.round(cp, 4))
        for name, Rc in (("as-is", np.eye(3)),
                         ("USD(-Z fwd, +Y up)", np.diag([1.0, -1.0, -1.0]))):
            w = cp + R.dot(Rc.dot(p_cam))
            print("  월드 추정 [%s] %s" % (name, np.round(w, 4)))
            if est_w is None:
                est_w = w

    ps = {}
    try:
        ps = backend.get_privileged_state()
    except Exception:
        pass
    truth = None
    for k in ps:
        if "can" in str(k).lower():
            vv = np.asarray(ps[k], dtype=np.float64).reshape(-1)
            if vv.size >= 3:
                truth = vv[:3]; break
    if truth is None:
        print("정답 캔 위치를 못 읽음"); return 4
    print("정답 캔 월드", np.round(truth, 4))
    if est_w is not None:
        for name, Rc in (("as-is", np.eye(3)),
                         ("USD(-Z fwd, +Y up)", np.diag([1.0, -1.0, -1.0]))):
            R = cr if (isinstance(cr, np.ndarray) and cr.shape == (3, 3)) else quat_to_R(cr)
            w = cp + R.dot(Rc.dot(p_cam))
            e = w - truth
            print("  오차 [%s] dx %+.4f dy %+.4f dz %+.4f | xy %.4f m"
                  % (name, e[0], e[1], e[2], float(np.linalg.norm(e[:2]))))
    print("\n캔 반지름 보정(CANRAD=%.3f)은 축 규약 확정 후 적용한다." % CANRAD)
    print("진단 이미지:", DUMP)
    return 0


if __name__ == "__main__":
    sys.exit(main())
