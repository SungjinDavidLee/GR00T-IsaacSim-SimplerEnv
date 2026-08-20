#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cam_rel.py -- wrist D405 가 panda_hand 기준으로 어디에 붙어 있는지 재고,
              LIBERO(robosuite) 의 wrist 카메라 장착 규격과 비교한다.

 왜 필요한가:
   GR00T 는 wrist 영상에서 캔이 특정 위치에 오도록 움직인다. 카메라가 학습 때와
   다른 곳에 붙어 있으면, 캔을 영상 중앙에 맞춰도 **그리퍼는 그 차이만큼 빗나간다.**
   XBIAS/YBIAS 가 흡수하고 있는 상수의 일부가 이것일 수 있다.
   이 값이 LIBERO 규격과 다르면, 상수로 때우는 대신 카메라를 옮기는 게 맞다.

 LIBERO / robosuite Panda 의 wrist(eye_in_hand) 카메라 규격:
   parent = robot0_eef(= panda_hand 등가),  pos = [0.05, 0, 0],
   quat(xyzw) = [0, 0.70711, 0.70711, 0],  fovy = 75
   ※ 이 값은 프로젝트 기록 기준이며, robosuite 원본 XML 로 재확인할 것 [확인 필요]

 실행:
   cd /home/data/groot/MIGRATE/ENV
   python3 /home/data/groot/work/cam_rel.py --config config/environment_groot.yaml
"""
import os, sys
if int(os.environ.get("NOWIN", "1")):
    sys.argv += ["--no-window"]
import numpy as np
import isaac_simpler_env_multi_object as base

HAND = os.environ.get("HANDPRIM", "/World/Franka/panda_hand")
CAM = os.environ.get("CAMPRIM", "/World/Franka/panda_hand/D405Mount/Camera_D405")


def xform(path):
    from pxr import UsdGeom, Usd
    import omni.usd
    st = omni.usd.get_context().get_stage()
    p = st.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return None, None
    M = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())).T
    R = M[:3, :3]
    R = R / np.maximum(np.linalg.norm(R, axis=0), 1e-12)
    return M[:3, 3].copy(), R


def R_to_quat_xyzw(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def main():
    cfg = base.CONFIG
    backend = base.IsaacBackend(cfg)
    task = base.LiftObjectTask(cfg)
    env = base.FrankaSimplerEnv(cfg, backend, task)
    env.reset()
    backend._require_ready()
    for _ in range(30):
        backend.advance_control_step(render=True)

    hp, hR = xform(HAND)
    cp, cR = xform(CAM)
    if hp is None or cp is None:
        print("prim 조회 실패", HAND, CAM); return 1

    print("panda_hand  world pos", np.round(hp, 4))
    print("D405 camera world pos", np.round(cp, 4))
    rel_p = hR.T.dot(cp - hp)
    rel_R = hR.T.dot(cR)
    print("\n== 손 프레임 기준 카메라 장착 ==")
    print("  pos        ", np.round(rel_p, 4))
    print("  quat(xyzw) ", np.round(R_to_quat_xyzw(rel_R), 5))
    print("  R\n", np.round(rel_R, 4))

    lib_p = np.array([0.05, 0.0, 0.0])
    print("\n== LIBERO wrist 규격과의 차 [확인 필요] ==")
    print("  LIBERO pos [0.05, 0, 0]   현재 %s" % np.round(rel_p, 4))
    print("  차이 %s   |xy| %.4f m" % (np.round(rel_p - lib_p, 4),
                                       float(np.linalg.norm((rel_p - lib_p)[:2]))))
    print("  LIBERO quat(xyzw) [0, 0.70711, 0.70711, 0]")
    print("\n해석: 이 차이가 XBIAS/YBIAS 크기(약 [0.19, 0.105] m)와 비슷하면")
    print("      상수의 상당 부분이 카메라 장착 불일치다 -> 카메라를 옮겨서 없앤다.")
    print("      훨씬 작으면 상수의 주된 원인은 체크포인트의 기억된 궤적이다")
    print("      -> 파인튜닝 없이는 못 없애고, 상수 보정이 정당한 대응이다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
