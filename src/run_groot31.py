"""
run_groot31.py  --  run_groot30.py 상위 호환 (위치 일반화용)

기본값은 run_groot30.py와 동일하게 동작한다. 새 기능은 전부 env로 opt-in.

 ── 새 env 스위치 ────────────────────────────────────────────────
 ANCHOR=follow|fixed   프레임 앵커. follow(기본)=캔 실제 위치로 OFF 재계산(=30번 동작),
                       fixed=고정 기준점으로 OFF 계산 → 캔이 움직여도 프레임 불변.
 CANREF="x,y,z"        ANCHOR=fixed 일 때 쓸 기준 캔 월드 좌표.
 OFFREF="x,y,z"        OFF를 통째로 직접 지정(최우선). 캔 privileged 조회 자체를 우회.
 NSUBOPEN=<int>        그리퍼가 열려 있는 동안(접근 구간) 실행할 청크 길이.
                       기본 = NSUB. 2로 낮추면 접근 중 정책 재질의 횟수가 4배 → 시각 되먹임 확보.
 CLOSEMIN=<int>        이 스텝 전의 '닫기' 명령은 무시(조기 파지 억제). 기본 0.
 ROT6=0|1              1이면 모델의 roll/pitch/yaw 출력을 자세 델타로 사용(6-DoF 복원). 실험용.
 RSCALE=<float>        회전 액션 스케일 (기본 0.05).
 RMAX=<float>          초기 자세 대비 누적 회전 상한 rad (기본 0.5). 발산 방지.
 PERTURB="s,dx,dy"     스텝 s 시작 시 캔을 (dx,dy) 만큼 순간이동. 재조준 여부 판정용.
 SBLIND=1              state의 x/y/z를 학습 시작값으로 고정해서 넣음(고유수용 차단).
 SNOISE=<float>        state x/y/z에 가우시안 노이즈(m) 주입.
 WBLIND=1              wrist 이미지를 회색으로(=wrist 차단). BLIND=1은 기존대로 front 차단.
 CLAMPLOG=1            작업공간 클램프가 실제로 걸린 스텝을 출력.
 TAG="..."             RESULT 줄에 붙일 식별자.

 ── 출력 ────────────────────────────────────────────────────────
 마지막에 기계 판독용 한 줄:
   RESULT tag=... ok=0/1 close_step=.. dx=.. dy=.. dz=.. fit=0/1 lift=.. canx=.. cany=..
        eex=.. eey=.. abort=0/1 nstep=..
"""
import os, sys, re
if int(os.environ.get("NOWIN", "0")):
    sys.argv += ["--no-window"]
if int(os.environ.get("FORCEGPU", "0")):
    sys.argv += ["--/renderer/activeGpu=0", "--/renderer/multiGpu/enabled=false"]
import numpy as np
import cv2
# --- patch31_det: ANCHOR=detect ---
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import isaac_simpler_env_multi_object as base
from isaacsim.core.utils.types import ArticulationAction
from groot_client import GrootClient

CONFIG = base.CONFIG
MAX_STEPS = int(os.environ.get('MAXSTEPS', '40'))
N_SUB = int(os.environ.get('NSUB', '8'))
N_SUB_OPEN = int(os.environ.get('NSUBOPEN', str(N_SUB)))
ACTION_SCALE = float(os.environ.get('ASCALE', '0.05'))
GRASP_OFFSET = 0.1034 + float(os.environ.get('GOFF', '0.018'))
LIB_OBJ = np.array([0.05, -0.10, 0.035])
LIB_EE0 = np.array([-0.1485, 0.0, 0.2613])
INSTRUCTION = os.environ.get("INSTR", "pick up the tomato sauce and place it in the basket")

# CAMMODE: both(기본) | wrist(front를 회색으로) | wristdup(front 슬롯에 wrist 복제)
#          | front(wrist를 회색으로)
CAMMODE = os.environ.get("CAMMODE", "both").lower()
try:
    FRONTPOS = ",".join("%.3f" % float(v)
                        for v in CONFIG["cameras"]["front"]["translate_xyz"])
except Exception:
    FRONTPOS = "?"

ANCHOR = os.environ.get("ANCHOR", "follow").lower()
CLOSEMIN = int(os.environ.get("CLOSEMIN", "0"))
ROT6 = int(os.environ.get("ROT6", "0"))
RSCALE = float(os.environ.get("RSCALE", "0.05"))
RMAX = float(os.environ.get("RMAX", "0.5"))
SBLIND = int(os.environ.get("SBLIND", "0"))
SNOISE = float(os.environ.get("SNOISE", "0"))
CLAMPLOG = int(os.environ.get("CLAMPLOG", "0"))
TAG = os.environ.get("TAG", "run")
# 캔 기본(형 원본) 배치의 월드 좌표. ANCHOR=fixed 기본 기준점.
CAN_NOMINAL = np.array([-0.0292, -0.0989, 0.9336])


def env_vec(name):
    v = os.environ.get(name, "")
    if not v:
        return None
    return np.array([float(t) for t in v.replace(" ", "").split(",")])


def ee_pose(backend):
    backend._sync_lula_robot_base_pose()
    p, r = backend._articulation_ik.compute_end_effector_pose()
    p = np.asarray(p, dtype=np.float64).reshape(3)
    R = np.asarray(r, dtype=np.float64).reshape(3, 3)
    q = np.asarray(base.rot_matrices_to_quats(R)).reshape(4)
    return p, R, q


def mat_to_axisangle(R):
    c = (np.trace(R) - 1.0) * 0.5
    c = float(np.clip(c, -1.0, 1.0))
    ang = float(np.arccos(c))
    if ang < 1e-8:
        return np.zeros(3)
    if abs(np.pi - ang) < 1e-4:
        A = (R + np.eye(3)) * 0.5
        ax = np.sqrt(np.clip(np.diag(A), 0.0, None))
        i = int(np.argmax(ax))
        v = A[:, i] / (ax[i] + 1e-12)
        v = v / (np.linalg.norm(v) + 1e-12)
        return v * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * np.sin(ang)) * ang


def axisangle_to_mat(v):
    """Rodrigues. v = axis*angle (rad)."""
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K.dot(K)


# --- patch31_obj: OBJ ---
OBJNAME = os.environ.get("OBJ", "can")


def find_obj(backend, pat):
    try:
        ps = backend.get_privileged_state()
    except Exception:
        return None
    for k in ps:
        if re.search(pat, str(k), re.I):
            try:
                v = np.asarray(ps[k], dtype=np.float64).reshape(-1)
                if v.size >= 3:
                    return v[:3]
            except Exception:
                pass
    return None


def to224(img):
    a = np.asarray(img)
    if a.ndim == 2:
        a = np.stack([a] * 3, axis=-1)
    if a.shape[2] == 4:
        a = a[:, :, :3]
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    return cv2.resize(a, (224, 224), interpolation=cv2.INTER_AREA)


def wprep(img):
    a = np.asarray(img)
    if a.ndim == 2:
        a = np.stack([a] * 3, axis=-1)
    if a.shape[2] == 4:
        a = a[:, :, :3]
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    if int(os.environ.get("WCROP", "1")):
        h, w = a.shape[:2]
        s = min(h, w)
        a = a[(h - s) // 2:(h - s) // 2 + s, (w - s) // 2:(w - s) // 2 + s]
    rot = int(os.environ.get("WROT", "180"))
    if rot:
        a = np.rot90(a, rot // 90)
    if int(os.environ.get("WFLIP", "0")):
        a = a[:, ::-1]
    a = np.ascontiguousarray(a)
    a = cv2.resize(a, (224, 224), interpolation=cv2.INTER_AREA)
    if int(os.environ.get("WBLIND", "0")):
        a = np.full_like(a, 128)
    return a


def fprep(img):
    a = np.asarray(img)
    if a.ndim == 2:
        a = np.stack([a] * 3, axis=-1)
    if a.shape[2] == 4:
        a = a[:, :, :3]
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    if int(os.environ.get("FCROP", "1")):
        h, w = a.shape[:2]
        s = min(h, w)
        a = a[(h - s) // 2:(h - s) // 2 + s, (w - s) // 2:(w - s) // 2 + s]
    a = np.ascontiguousarray(a)
    a = cv2.resize(a, (224, 224), interpolation=cv2.INTER_AREA)
    if int(os.environ.get("BLIND", "0")):
        a = np.full_like(a, 128)
    return a


def set_gripper(backend, robot, opened):
    t = np.array([0.04, 0.04] if opened else [0.0, 0.0], dtype=np.float32)
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=t, joint_indices=backend.FINGER_DOF_INDICES))


def teleport_can(backend, dxy):
    """PERTURB용. 캔을 xy로 순간이동.

    ★ v2 수정: UsdGeom XformOp 를 직접 건드리면 PhysX 가 그 변위를 속도로 해석해
    캔을 날려버린다(8/18 perturb 로그에서 캔이 1스텝 만에 6.5 cm 튀고 넘어짐).
    SingleRigidPrim.set_world_pose + 속도 0 리셋을 써야 한다.
    """
    try:
        prim = backend.objects[OBJNAME]
    except Exception as e:
        print("PERTURB unavailable (backend.objects[OBJNAME]):", e)
        return None
    pos, quat = prim.get_world_pose()
    pos = np.asarray(pos, dtype=np.float64).reshape(3).copy()
    new = pos + np.array([float(dxy[0]), float(dxy[1]), 0.0])
    prim.set_world_pose(position=new.astype(np.float32),
                        orientation=np.asarray(quat, dtype=np.float32))
    prim.set_linear_velocity(np.zeros(3, dtype=np.float32))
    prim.set_angular_velocity(np.zeros(3, dtype=np.float32))
    print("PERTURB %s -> %s (cmd +[%.3f %.3f])"
          % (np.round(pos, 4), np.round(new, 4), dxy[0], dxy[1]))
    return new


def main():
    backend = base.IsaacBackend(CONFIG)
    task = base.LiftObjectTask(CONFIG)
    env = base.FrankaSimplerEnv(CONFIG, backend, task)
    env.reset()
    robot, _ = backend._require_ready()

    # --- patch31_cam: CAMSHIFT ---
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

    bp, bq = robot.get_world_pose()
    bp = np.asarray(bp, dtype=np.float64).reshape(3)
    w, x, y, z = [float(v) for v in np.asarray(bq).reshape(4)]
    Rb = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float64)
    RbT = Rb.T

    # ---- 캔 실제 위치 (평가/로깅 전용. 정책 입력에는 ANCHOR=fixed일 때 안 쓰임) ----
    can_w = find_obj(backend, OBJNAME)
    if can_w is None:
        can_w = CAN_NOMINAL.copy()

    # ---- 프레임 앵커 결정 ----
    bias = np.array([float(os.environ.get('XBIAS', '0')),
                     float(os.environ.get('YBIAS', '0')),
                     float(os.environ.get('ZBIAS', '0'))])
    det_err = np.array([0.0, 0.0, 0.0])   # # --- patch31_det: ANCHOR=detect ---
    off_direct = env_vec("OFFREF")
    canref = env_vec("CANREF")
    if off_direct is not None:
        OFF = off_direct.copy()
        anchor_w = None
        print("ANCHOR=offref  OFF given directly", np.round(OFF, 4))
    elif ANCHOR.startswith("det"):
        # # --- patch31_det: ANCHOR=detect ---
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
    elif ANCHOR.startswith("fix"):
        anchor_w = canref if canref is not None else CAN_NOMINAL.copy()
        OFF = RbT.dot(anchor_w - bp) - LIB_OBJ - bias
        print("ANCHOR=fixed   ref", np.round(anchor_w, 4),
              " actual can", np.round(can_w, 4),
              " delta", np.round(can_w - anchor_w, 4))
    else:
        anchor_w = can_w.copy()
        OFF = RbT.dot(can_w - bp) - LIB_OBJ - bias
        print("ANCHOR=follow  (캔 실제 위치로 프레임 재계산 — 위치가 변수에서 소거됨)")
    print("can world", np.round(can_w, 4),
          "base", np.round(RbT.dot(can_w - bp), 4))
    print("OFFSET", np.round(OFF, 4), " bias", np.round(bias, 4))

    # ---- 작업공간 클램프 상자 ----
    # 주의: lp(보고되는 state) = 참 LIBERO 좌표 + bias 다. 따라서 기본 상자를
    # 그대로 쓰면 물리적으로는 bias 만큼 밀린 영역을 강제하게 된다.
    #   CLAMPFRAME=biased(기본)  8/17 확정 설정과 동일
    #   CLAMPFRAME=true          참 LIBERO 좌표에 상자를 걸음 (상자를 +bias 만큼 이동)
    #   CLAMPPAD=<m>             xy 상자를 사방으로 넓힘
    #   CLAMPOFF=1               xy 클램프 해제 (z 범위와 z_floor 는 유지)
    CLO = np.array([-0.176, -0.294, 0.010])
    CHI = np.array([0.146, float(os.environ.get('YMAX', '0.332')), 0.386])
    if os.environ.get("CLAMPFRAME", "biased").lower().startswith("tru"):
        CLO = CLO + bias
        CHI = CHI + bias
    _pad = float(os.environ.get("CLAMPPAD", "0"))
    if _pad:
        CLO = CLO - np.array([_pad, _pad, 0.0])
        CHI = CHI + np.array([_pad, _pad, 0.0])
    if int(os.environ.get("CLAMPOFF", "0")):
        CLO[0] = CLO[1] = -1e3
        CHI[0] = CHI[1] = 1e3
    print("CLAMP BOX lo %s hi %s  (frame=%s pad=%s off=%s)" % (
        np.round(CLO, 3), np.round(CHI, 3),
        os.environ.get("CLAMPFRAME", "biased"), _pad,
        os.environ.get("CLAMPOFF", "0")))

    ee_goal_b = LIB_EE0 + OFF
    ee_goal_w = bp + Rb.dot(ee_goal_b) + np.array([0.0, 0.0, GRASP_OFFSET])
    p0, R0, q0 = ee_pose(backend)
    print("GOFF", os.environ.get("GOFF", "0.018"),
          "GRASP_OFFSET %.4f" % GRASP_OFFSET)
    print("ZSCALE", os.environ.get("ZSCALE", "1.0"),
          "FLOORB", os.environ.get("FLOORB", "-0.02"))
    print("ASCALE", ACTION_SCALE, "NSUB", N_SUB, "NSUBOPEN", N_SUB_OPEN,
          "CLOSEMIN", CLOSEMIN, "ROT6", ROT6,
          "GHOLD", os.environ.get("GHOLD", "0"),
          "XBIAS", bias[0], "YBIAS", bias[1])
    print("CAMMODE", CAMMODE, "| front D455 translate_xyz", FRONTPOS)
    print("INSTR:", INSTRUCTION)
    print("ee now", np.round(p0, 4), "-> goal", np.round(ee_goal_w, 4))

    okc = 0
    for i in range(120):
        t = float(i + 1) / 120.0
        pi = p0 + (ee_goal_w - p0) * t
        backend._sync_lula_robot_base_pose()
        a, ok = backend._articulation_ik.compute_inverse_kinematics(
            target_position=pi, target_orientation=q0,
            position_tolerance=0.005, orientation_tolerance=0.05)
        if ok:
            okc += 1
            robot.apply_action(a)
        backend.advance_control_step(render=True)
    set_gripper(backend, robot, True)
    for _ in range(30):
        backend.advance_control_step(render=True)

    p0, R0, q0 = ee_pose(backend)
    print("HOMING ik %d/120 -> %s err %.4f" % (okc, np.round(p0, 4),
          float(np.linalg.norm(p0 - ee_goal_w))))
    lp0 = RbT.dot(p0 - np.array([0.0, 0.0, GRASP_OFFSET]) - bp) - OFF
    print("LIBERO-frame start", np.round(lp0, 4), " target", LIB_EE0)

    # z_floor: ANCHOR=fixed면 기준점 높이를 쓴다 (캔 실제 높이 누설 차단)
    # # --- patch31_det: ANCHOR=detect ---  detect 모드도 추정 높이를 쓴다 (정답 높이 누설 차단)
    z_ref = float(anchor_w[2]) \
        if (anchor_w is not None and ANCHOR[:3] in ("fix", "det")) \
        else float(can_w[2])
    z_floor = z_ref + GRASP_OFFSET + float(os.environ.get('FLOORB', '-0.02'))
    print('FLOORB', os.environ.get('FLOORB', '-0.02'), 'z_floor %.4f' % z_floor)
    z_can0 = float(can_w[2])

    imgs = backend.get_camera_images()
    print("CAMERA KEYS", sorted([str(k) for k in imgs]))
    fk = "front" if "front" in imgs else sorted(imgs)[0]
    wk = "wrist" if "wrist" in imgs else sorted(imgs)[-1]

    client = GrootClient()
    print("PING:", client.call("ping"))
    client.reset()

    # --- patch31: video+input-dump ---
    VIDDIR = os.environ.get("VIDDIR", "")
    VIDEVERY = max(1, int(os.environ.get("VIDEVERY", "1")))
    DBG = VIDDIR if VIDDIR else "/home/data/groot/work"
    vidn = [0]      # 기록된 프레임 수
    snapi = [0]     # snap() 호출 수
    if VIDDIR:
        os.makedirs(VIDDIR, exist_ok=True)
        for _f in os.listdir(VIDDIR):
            if _f.endswith(".png"):
                os.remove(os.path.join(VIDDIR, _f))
    # 정책에 실제로 들어간 224 입력(CAMMODE 적용 후)을 snap() 이 읽는다.
    # 이게 없으면 CAMMODE 처치가 걸렸는지 영상만 보고 확인할 수 없다.
    last_in = {"f": None, "w": None, "step": -1}

    def _lab(img, txt, y, dark=False):
        cv2.putText(img, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def snap():
        if not VIDDIR:
            return
        i = snapi[0]
        snapi[0] += 1
        if i % VIDEVERY:
            return
        im = backend.get_camera_images()
        f1 = np.asarray(im[fk])[:, :, :3]
        w1 = np.asarray(im[wk])[:, :, :3]
        if f1.dtype != np.uint8:
            f1 = np.clip(f1, 0, 255).astype(np.uint8)
        if w1.dtype != np.uint8:
            w1 = np.clip(w1, 0, 255).astype(np.uint8)
        top = np.hstack([cv2.resize(f1, (480, 360)),
                         cv2.resize(w1, (480, 360))])

        def _pan(a):
            if a is None:
                return np.zeros((360, 480, 3), np.uint8)
            return cv2.resize(np.ascontiguousarray(a), (480, 360),
                              interpolation=cv2.INTER_NEAREST)
        bot = np.hstack([_pan(last_in["f"]), _pan(last_in["w"])])
        both = np.ascontiguousarray(np.vstack([top, bot]))
        both = cv2.cvtColor(both, cv2.COLOR_RGB2BGR)
        _lab(both, "RAW front (%s)" % FRONTPOS, 22)
        _lab(both, "RAW wrist", 22)
        cv2.putText(both, "RAW wrist", (488, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(both, "RAW wrist", (488, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (255, 255, 255), 1, cv2.LINE_AA)
        for _x, _t in ((8, "MODEL IN image (224)"), (488, "MODEL IN wrist_image (224)")):
            cv2.putText(both, _t, (_x, 382), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(both, _t, (_x, 382), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (255, 255, 255), 1, cv2.LINE_AA)
        _lab(both, "tag=%s cam=%s s%03d f%05d" % (
            TAG, CAMMODE, last_in["step"], vidn[0]), 712)
        cv2.imwrite(os.path.join(VIDDIR, "f%05d.png" % vidn[0]), both)
        vidn[0] += 1

    target_p = p0.copy()
    target_q = q0.copy()
    Rt_b = RbT.dot(R0)          # ROT6: base 프레임 목표 자세
    R0_b = Rt_b.copy()
    grip_open = True
    ccount = 0
    GDEB = int(os.environ.get('GDEB', '3'))
    hold = 0
    GHOLD = int(os.environ.get('GHOLD', '0'))
    # --- patch31_latch: NOREOPEN ---
    NOREOPEN = int(os.environ.get('NOREOPEN', '0'))
    latched = [False]
    print("NOREOPEN", NOREOPEN, "(1이면 첫 파지 후 열기 명령 무시)")
    rng = np.random.default_rng(0)

    perturb = env_vec("PERTURB")
    # # --- patch31_cam: CAMSHIFT ---  FIT 은 dy 만 보고 있었다. dx 가 34 mm 어긋나도 FIT=1 이 찍혔다.
    FITDY = float(os.environ.get("FITDY", "0.0063"))
    FITDX = float(os.environ.get("FITDX", "0.015"))
    FITDZ = float(os.environ.get("FITDZ", "0.020"))
    print("FIT 기준  |dx|<%.4f  |dy|<%.4f  |dz|<%.4f" % (FITDX, FITDY, FITDZ))
    res = dict(ok=0, close_step=-1, dx=9.99, dy=9.99, dz=9.99, fit=0,
               lift=0.0, abort=0, nstep=0, nclose=0, last_dy=9.99)
    clamp_tot = np.zeros(3, dtype=float)
    clamp_n = 0

    for step in range(MAX_STEPS):
        if perturb is not None and step == int(perturb[0]):
            want = teleport_can(backend, perturb[1:3])
            # 잔여 임펄스 제거: 처음 몇 스텝은 속도를 계속 0으로 눌러 안정화
            for _s in range(30):
                if _s < 6:
                    try:
                        backend.objects[OBJNAME].set_linear_velocity(
                            np.zeros(3, dtype=np.float32))
                        backend.objects[OBJNAME].set_angular_velocity(
                            np.zeros(3, dtype=np.float32))
                    except Exception:
                        pass
                backend.advance_control_step(render=True)
            can_w = find_obj(backend, OBJNAME)
            print("PERTURB new can world", np.round(can_w, 4))
            if want is not None:
                err = float(np.linalg.norm(np.asarray(can_w)[:2] - want[:2]))
                print("PERTURB placement err %.4f m %s" % (
                    err, "OK" if err < 0.005 else "!! 이동이 깨끗하지 않음 — 판정 무효"))
            z_can0 = float(can_w[2])

        imgs = backend.get_camera_images()
        ep, eR, _ = ee_pose(backend)
        ep = ep + eR.dot(np.array([0.0, 0.0, GRASP_OFFSET]))
        lp = RbT.dot(ep - bp) - OFF
        aa = mat_to_axisangle(RbT.dot(eR))
        if aa[0] < 0:
            aa = -aa
        jq = np.asarray(robot.get_joint_positions()).reshape(-1)
        fi = list(backend.FINGER_DOF_INDICES)
        gr = np.array([jq[fi[0]], -jq[fi[1]]], dtype=np.float32)

        lp_in = lp.copy()
        if SBLIND:
            lp_in = lp0.copy()
        if SNOISE > 0:
            lp_in = lp_in + rng.normal(0.0, SNOISE, 3)

        def s(v):
            return np.array([[[float(v)]]], dtype=np.float32)

        _f224 = fprep(imgs[fk])
        _w224 = wprep(imgs[wk])
        if CAMMODE.startswith("wristdup"):
            _f224 = _w224.copy()
        elif CAMMODE.startswith("wrist"):
            _f224 = np.full_like(_f224, 128)
        elif CAMMODE.startswith("front"):
            _w224 = np.full_like(_w224, 128)
        # # --- patch31: video+input-dump ---  영상에 '모델이 실제로 본 것'을 남긴다
        last_in["f"], last_in["w"], last_in["step"] = _f224, _w224, step
        snap()

        gobs = {
            "video": {
                "image": _f224[None, None],
                "wrist_image": _w224[None, None],
            },
            "state": {
                "x": s(lp_in[0]), "y": s(lp_in[1]), "z": s(lp_in[2]),
                "roll": s(aa[0]), "pitch": s(aa[1]), "yaw": s(aa[2]),
                "gripper": gr.reshape(1, 1, 2),
            },
            "language": {
                "annotation.human.action.task_description": [[INSTRUCTION]],
            },
        }
        if step == 0:
            cv2.imwrite(os.path.join(DBG, "f0.png"),
                        cv2.cvtColor(gobs["video"]["image"][0, 0], cv2.COLOR_RGB2BGR))
            cv2.imwrite(os.path.join(DBG, "w0.png"),
                        cv2.cvtColor(gobs["video"]["wrist_image"][0, 0], cv2.COLOR_RGB2BGR))
            for _k in list(imgs):
                if str(_k).endswith("depth"):
                    continue
                cv2.imwrite(os.path.join(DBG, "cam_" + str(_k) + ".png"),
                            cv2.cvtColor(to224(imgs[_k]), cv2.COLOR_RGB2BGR))
            print("WRIST MODE crop=%s rot=%s flip=%s  BLIND f=%s w=%s SBLIND=%s" % (
                os.environ.get("WCROP", "1"), os.environ.get("WROT", "180"),
                os.environ.get("WFLIP", "0"), os.environ.get("BLIND", "0"),
                os.environ.get("WBLIND", "0"), SBLIND))
        print("STATE s%03d xyz %s rpy %s grip %s" % (
            step, np.round(lp_in, 3), np.round(aa, 3), np.round(gr, 4)))

        # --- patch31_det2: POLICYCHK ---
        _pk = int(os.environ.get("POLICYCHK", "0"))
        if _pk > 1 and step == 0:
            import copy
            _chs = []
            for _i in range(_pk):
                _a = client.get_action(copy.deepcopy(gobs))
                _c = np.concatenate(
                    [np.asarray(_a[_k]).reshape(-1)
                     for _k in sorted(_a) if hasattr(_a[_k], "__len__")])
                _chs.append(_c)
                print("POLICYCHK call%d  head8 %s" % (
                    _i, np.array2string(_c[:8], precision=5,
                                        suppress_small=False)))
            _M = np.stack(_chs)
            _sd = _M.std(axis=0)
            _rg = _M.max(axis=0) - _M.min(axis=0)
            print("POLICYCHK n=%d dim=%d  std 평균 %.6f 최대 %.6f | 범위 평균 %.6f 최대 %.6f"
                  % (_pk, _M.shape[1], _sd.mean(), _sd.max(),
                     _rg.mean(), _rg.max()))
            if _sd.max() < 1e-6:
                print("POLICYCHK 판정: **결정적** — 같은 관측에 같은 행동. "
                      "변동은 시뮬레이터 쪽이다.")
            else:
                print("POLICYCHK 판정: **확률적** — 같은 관측에 다른 행동. "
                      "정책 샘플링이 변동의 원인이다. 서버에서 시드를 고정해야 "
                      "조건 비교가 성립한다.")
        # --- patch31_ksamp: K-sample median ---
        _K = int(os.environ.get("KSAMP", "1"))
        if grip_open:
            _K = int(os.environ.get("KSAMPOPEN", str(_K)))
        if _K > 1:
            import copy as _cp
            _samples = [client.get_action(_cp.deepcopy(gobs)) for _ in range(_K)]
            act = {}
            for _k in _samples[0]:
                try:
                    _st = np.stack([np.asarray(_s[_k], dtype=np.float64)
                                    for _s in _samples])
                except Exception:
                    act[_k] = _samples[0][_k]
                    continue
                act[_k] = np.median(_st, axis=0)
                if int(os.environ.get("KLOG", "0")) and step < 6:
                    print("KSAMP s%03d %-16s K=%d std %.5f -> 중앙값 잔여 %.5f"
                          % (step, str(_k), _K, _st.std(axis=0).mean(),
                             _st.std(axis=0).mean() / max(_K ** 0.5, 1.0)))
        else:
            act = client.get_action(gobs)
        gsig = -np.sign(2.0 * np.asarray(act["gripper"]).reshape(-1) - 1.0)
        # --- patch31_zero: ZEROACT ---
        _za = int(os.environ.get("ZEROACT", "0"))
        ch = np.stack([
            np.asarray(act["x"]).reshape(-1),
            np.asarray(act["y"]).reshape(-1),
            np.asarray(act["z"]).reshape(-1),
            gsig], axis=1)
        rot = None
        if ROT6:
            try:
                rot = np.stack([np.asarray(act["roll"]).reshape(-1),
                                np.asarray(act["pitch"]).reshape(-1),
                                np.asarray(act["yaw"]).reshape(-1)], axis=1)
            except Exception as e:
                print("ROT6: action has no roll/pitch/yaw ->", e)
                rot = None
        if _za == 1:                     # # --- patch31_zero: ZEROACT ---
            ch[:, :3] = 0.0
            if step == 0:
                print("ZEROACT=1  정책 병진 출력을 0으로 (대조 실험)")
        elif _za == 2:
            _rs = np.random.default_rng(1234 + step)
            _sd = float(np.abs(ch[:, :3]).mean()) + 1e-9
            ch[:, :3] = _rs.normal(0.0, _sd, size=ch[:, :3].shape)
            if step == 0:
                print("ZEROACT=2  정책 병진 출력을 무작위로 대체 (대조 실험)")
        _c3 = ch[:, :3].copy()
        _c3[:, 2] = _c3[:, 2] * float(os.environ.get('ZSCALE', '1.0'))
        d = (_c3 * ACTION_SCALE).dot(RbT)
        _vx = np.asarray(act["x"]).reshape(-1)
        _vy = np.asarray(act["y"]).reshape(-1)
        _vz = np.asarray(act["z"]).reshape(-1)
        _vg = np.asarray(act["gripper"]).reshape(-1)
        print("ACT s%03d x %+.3f y %+.3f z %+.3f g %+.3f ystd %.3f%s" % (
            step, _vx.mean(), _vy.mean(), _vz.mean(), _vg.mean(), _vy.std(),
            ("" if rot is None else "  rpy %s" % np.round(rot.mean(axis=0), 4))))
        if step < 8 or step % 10 == 0:
            cv2.imwrite(os.path.join(DBG, "seq_w%03d.png" % step),
                        cv2.cvtColor(gobs["video"]["wrist_image"][0, 0],
                                     cv2.COLOR_RGB2BGR))

        if int(os.environ.get("RESEED", "0")):
            target_p = ee_pose(backend)[0]
        _lp_pre = lp.copy()
        # ★ 접근 구간(그리퍼 열림)에서는 청크를 짧게 실행 → 정책 재질의 빈도 상승
        nuse = N_SUB_OPEN if grip_open else N_SUB
        nuse = min(nuse, d.shape[0])
        _cmd_l = RbT.dot(d[:nuse].sum(axis=0))
        ik = 0
        clamped = np.zeros(3, dtype=int)
        for k in range(nuse):
            target_p = target_p + d[k]
            _g = np.array([0.0, 0.0, GRASP_OFFSET])
            _l = RbT.dot(target_p - _g - bp) - OFF
            _lo = _l.copy()
            _l = np.minimum(np.maximum(_l, CLO), CHI)
            clamped += (np.abs(_l - _lo) > 1e-9).astype(int)
            target_p = bp + Rb.dot(_l + OFF) + _g
            if target_p[2] < z_floor:
                target_p[2] = z_floor

            if rot is not None:
                dr = rot[k] * RSCALE
                Rt_b = axisangle_to_mat(dr).dot(Rt_b)
                dev = mat_to_axisangle(R0_b.T.dot(Rt_b))
                n = float(np.linalg.norm(dev))
                if n > RMAX:
                    Rt_b = R0_b.dot(axisangle_to_mat(dev / n * RMAX))
                target_q = np.asarray(
                    base.rot_matrices_to_quats(Rb.dot(Rt_b))).reshape(4)

            backend._sync_lula_robot_base_pose()
            a, ok = backend._articulation_ik.compute_inverse_kinematics(
                target_position=target_p, target_orientation=target_q,
                position_tolerance=0.01, orientation_tolerance=0.05)
            if ok:
                ik += 1
                if int(os.environ.get("ARMONLY", "1")):
                    _jp = np.asarray(a.joint_positions).reshape(-1)
                    _ai = np.asarray(backend._arm_joint_indices).reshape(-1)
                    if _jp.size > _ai.size:
                        _jp = _jp[_ai]
                    robot.apply_action(ArticulationAction(
                        joint_positions=_jp, joint_indices=_ai))
                else:
                    robot.apply_action(a)
            else:
                target_p = ee_pose(backend)[0]
            g = -float(ch[k, 3])
            ccount = ccount + 1 if g < -0.3 else 0
            if hold > 0:
                hold -= 1
            elif grip_open and g < -0.3 and ccount + 1 >= GDEB and step >= CLOSEMIN:
                ccount = 0
                grip_open = False
                hold = GHOLD
                if NOREOPEN:
                    latched[0] = True          # # --- patch31_latch: NOREOPEN ---
                set_gripper(backend, robot, False)
                _ep2, _eR2, _ = ee_pose(backend)
                _ep2 = _ep2 + _eR2.dot(np.array([0.0, 0.0, GRASP_OFFSET]))
                _cn2 = find_obj(backend, OBJNAME)
                if _cn2 is not None:
                    _e2 = RbT.dot(_ep2 - np.asarray(_cn2))
                    fit = (abs(_e2[1]) < FITDY and abs(_e2[0]) < FITDX
                           and abs(_e2[2]) < FITDZ)
                    print("CLOSE s%03d dy %+.4f | dx %+.4f dz %+.4f | %s"
                          % (step, _e2[1], _e2[0], _e2[2],
                             "FIT" if fit else "HITS FINGER"))
                    # ★ 첫 파지만 기록한다. 실패 후 재시도(극한주기)의 마지막 값을
                    #   덮어쓰면 지표가 오염된다 — 8/18 base 스윕에서 실제로 발생.
                    res["nclose"] += 1
                    res["last_dy"] = float(_e2[1])
                    if res["close_step"] < 0:
                        res.update(close_step=step, dx=float(_e2[0]),
                                   dy=float(_e2[1]), dz=float(_e2[2]),
                                   fit=int(fit))
            elif grip_open and g < -0.3 and step < CLOSEMIN and k == 0:
                print("CLOSE suppressed s%03d (CLOSEMIN=%d)" % (step, CLOSEMIN))
            elif (not grip_open) and g > 0.3 and not latched[0]:
                grip_open = True
                set_gripper(backend, robot, True)
                print("GRIP OPEN  s%03d k%d" % (step, k))
            elif (not grip_open) and g > 0.3 and latched[0] and k == 0:
                print("GRIP OPEN ignored s%03d (NOREOPEN)" % step)
            for _st in range(int(os.environ.get("SETTLE", "8"))):
                backend.advance_control_step(render=True)
            snap()

        clamp_tot += clamped
        clamp_n += nuse
        if CLAMPLOG and clamped.sum():
            print("CLAMP s%03d xyz hits %s / %d" % (step, clamped.tolist(), nuse))

        ep, _eR2, _ = ee_pose(backend)
        ep = ep + _eR2.dot(np.array([0.0, 0.0, GRASP_OFFSET]))
        lp = RbT.dot(ep - bp) - OFF
        cn = find_obj(backend, OBJNAME)
        zc = float(cn[2]) if cn is not None else -1.0
        res["nstep"] = step + 1
        if cn is not None and int(os.environ.get("ABORT", "1")):
            _dxy = float(np.linalg.norm(np.asarray(cn)[:2] - can_w[:2]))
            if (zc - z_can0) < 0.03 and (_dxy > 0.03 or (z_can0 - zc) > 0.02):
                print("CAN DISTURBED s%03d dxy %.3f dz %+.3f -> ABORT"
                      % (step, _dxy, zc - z_can0))
                res["abort"] = 1
                break
        _ach = lp - _lp_pre
        _r = _ach[2] / _cmd_l[2] if abs(_cmd_l[2]) > 1e-6 else 0.0
        print("TRK s%03d n%d cmdz %+.4f achz %+.4f ratio %.2f" % (
            step, nuse, _cmd_l[2], _ach[2], _r))
        _e = RbT.dot(ep - can_w)
        print("ERR s%03d dx %+.3f dy %+.3f dz %+.3f  norm %.3f  %s" % (
            step, _e[0], _e[1], _e[2], float(np.linalg.norm(_e)),
            "O" if grip_open else "C"))
        if cn is not None:
            _cl = RbT.dot(np.asarray(cn) - bp) - OFF
            print("CANPOS s%03d W[%+.4f %+.4f %+.4f] L[%+.3f %+.3f %+.3f] dz %+.4f %s"
                  % (step, cn[0], cn[1], cn[2], _cl[0], _cl[1], _cl[2],
                     zc - z_can0, "O" if grip_open else "C"))
        if (not int(os.environ.get("NOBREAK", "0"))) and \
                zc - z_can0 > float(os.environ.get("LIFTTH", "0.08")):
            print("=== GRASP+LIFT at step", step, "dz %.4f ===" % (zc - z_can0))
            for _h in range(60):
                backend.advance_control_step(render=True)
                if _h % 10 == 0:
                    snap()
            _c3 = find_obj(backend, OBJNAME)
            _z3 = float(_c3[2]) if _c3 is not None else -1.0
            _j3 = np.asarray(robot.get_joint_positions()).reshape(-1)
            print("HOLD CHECK canz %.4f dz %.4f finger %s" % (
                _z3, _z3 - z_can0,
                np.round(_j3[list(backend.FINGER_DOF_INDICES)], 4)))
            res["lift"] = float(_z3 - z_can0)
            res["ok"] = 1 if (_z3 - z_can0) > 0.05 else 0
            print("=== SUCCESS ===" if res["ok"] else "=== DROPPED AFTER LIFT ===")
            break

    ep_f, _, _ = ee_pose(backend)
    cs = clamp_tot / max(clamp_n, 1)
    print(("RESULT tag=%s ok=%d close_step=%d dx=%+.4f dy=%+.4f dz=%+.4f "
           "fit=%d lift=%.4f canx=%+.4f cany=%+.4f eex=%+.4f eey=%+.4f "
           "abort=%d nstep=%d clampx=%.2f clampy=%.2f clampz=%.2f "
           "nclose=%d last_dy=%+.4f cammode=%s frontpos=%s "
           "detx=%+.4f dety=%+.4f detz=%+.4f anchor=%s camshift=%s") % (
        TAG, res["ok"], res["close_step"], res["dx"], res["dy"], res["dz"],
        res["fit"], res["lift"], can_w[0], can_w[1], ep_f[0], ep_f[1],
        res["abort"], res["nstep"], cs[0], cs[1], cs[2],
        res["nclose"], res["last_dy"], CAMMODE, FRONTPOS,
        det_err[0], det_err[1], det_err[2], ANCHOR,
        os.environ.get("CAMSHIFT", "0,0,0").replace(" ", "")))

    for _ in range(int(os.environ.get('IDLE', '200'))):
        backend.idle_step()
    return 0


if __name__ == "__main__":
    sys.exit(main())
