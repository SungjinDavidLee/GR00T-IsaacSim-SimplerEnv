#!/usr/bin/env python3
"""
front_mode.py -- front D455 배치 전환 + agentview 위치 재계산

  python3 front_mode.py agentview   # LIBERO agentview 등가 위치 (현재 오프셋으로 재계산)
  python3 front_mode.py legacy      # 8/17 성공 당시 fix_front.py 가 쓴 값 (A/B 비교용)
  python3 front_mode.py original    # D455 원위치 [-0.30,-0.024,1.42] rpy[0,-45,180]
  python3 front_mode.py show        # 현재 yaml 값과 세 후보를 출력만 (수정 없음)

── 왜 재계산이 필요한가 ─────────────────────────────────────────
fix_front.py 는 OFF = [0.585, -0.0899, -0.157] 로 카메라 위치를 잡았다.
그런데 현재 씬(캔 원배치, base [0.6268,-0.0237,1.0254], Rb=diag(-1,-1,1))에서
바이어스를 뺀 참 오프셋은

    OFF_true = RbT·(can_world - base) - LIB_OBJ = [0.606, 0.1752, -0.1268]

이다. y 성분이 0.175 vs -0.090 으로 약 0.265 m 어긋난다.
씬 좌표→월드 매핑은 XBIAS/YBIAS(상태 보정)와 무관해야 하므로,
카메라 배치에는 바이어스를 뺀 OFF_true 를 써야 한다.  [확인 필요 → A/B로 판정]

y 를 읽을 수 있는 유일한 센서가 front 이므로, 이 값이 틀려 있으면
캔이 y로 움직여도 모델이 그것을 위치 변화로 해석하지 못한다.
"""
import sys
import numpy as np

P = '/home/data/groot/MIGRATE/ENV/config/environment_groot.yaml'

BASE = np.array([0.6268, -0.0237, 1.0254])
Rb = np.diag([-1.0, -1.0, 1.0])
RbT = Rb.T
CAN_NOMINAL = np.array([-0.0292, -0.0989, 0.9336])
LIB_OBJ = np.array([0.05, -0.10, 0.035])

OFF_TRUE = RbT.dot(CAN_NOMINAL - BASE) - LIB_OBJ      # 바이어스 제외
OFF_LEGACY = np.array([0.585, -0.0899, -0.157])       # fix_front.py 가 썼던 값

CAM_LIB = np.array([0.89658, 0.0, 0.65])              # LIBERO agentview pos
Q_LIB = np.array([0.61822, 0.34323, 0.34323, 0.61822])  # (w,x,y,z)

ORIGINAL = (np.array([-0.30, -0.024, 1.42]), np.array([0.0, -45.0, 180.0]))


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def agentview_pose(off):
    pos_w = BASE + Rb.dot(CAM_LIB + off)
    R = Rb.dot(quat_to_mat(Q_LIB))
    pitch = np.arcsin(np.clip(-R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return pos_w, np.degrees([roll, pitch, yaw])


def read_front():
    s = open(P).read()
    i = s.index("  front:")
    j = s.index("translate_xyz:", i)
    k = s.index("resolution:", j)
    return s, i, j, k


def write_front(pos, rpy):
    s, i, j, k = read_front()
    blk = ("translate_xyz:\n    - %.4f\n    - %.4f\n    - %.4f\n"
           "    orientation_rpy_deg:\n    - %.2f\n    - %.2f\n    - %.2f\n    "
           % (pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]))
    open(P + ".frontbak", "w").write(s)
    open(P, "w").write(s[:j] + blk + s[k:])
    print("front -> pos %s  rpy %s" % (np.round(pos, 4).tolist(),
                                       np.round(rpy, 2).tolist()))
    print("backup:", P + ".frontbak")


def main():
    mode = (sys.argv[1] if len(sys.argv) > 1 else "show").lower()
    pa, ra = agentview_pose(OFF_TRUE)
    pl, rl = agentview_pose(OFF_LEGACY)
    print("OFF_true   ", np.round(OFF_TRUE, 4).tolist())
    print("OFF_legacy ", np.round(OFF_LEGACY, 4).tolist(),
          " diff", np.round(OFF_TRUE - OFF_LEGACY, 4).tolist())
    print("agentview(true)   pos %s rpy %s" % (np.round(pa, 4).tolist(),
                                               np.round(ra, 2).tolist()))
    print("agentview(legacy) pos %s rpy %s" % (np.round(pl, 4).tolist(),
                                               np.round(rl, 2).tolist()))
    print("original          pos %s rpy %s" % (ORIGINAL[0].tolist(),
                                               ORIGINAL[1].tolist()))
    s, i, _, _ = read_front()
    print("\n-- 현재 yaml front 블록 --\n" + s[i:i + 300])

    if mode == "agentview":
        write_front(pa, ra)
    elif mode == "legacy":
        write_front(pl, rl)
    elif mode == "original":
        write_front(*ORIGINAL)
    elif mode == "show":
        print("(show 모드 — yaml 미수정)")
    else:
        print("unknown mode:", mode)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
