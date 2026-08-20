#!/usr/bin/env python3
"""
pos_grid.py -- 캔 위치 그리드 스윕 하네스

environment_groot.yaml 의 can.translate_xy 를 격자로 옮겨가며
run_groot31.py 를 반복 실행하고, RESULT 줄을 파싱해 CSV로 남긴다.
yaml 은 시작 시 백업하고 종료(정상/Ctrl-C/예외) 시 반드시 복원한다.

사용 예)
  # 환경변수는 호출하는 쉘에서 export 해둔 것을 그대로 물려받는다
  python3 pos_grid.py --dx -0.06,-0.03,0,0.03,0.06 \
                      --dy -0.06,-0.03,0,0.03,0.06 \
                      --reps 3 --tag fixframe --out /home/data/groot/work/grid_fix.csv

주의: dx/dy 는 yaml 의 can.translate_xy 에 더하는 값(= 씬 좌표)이다.
"""
import argparse, csv, glob, os, re, signal, subprocess, sys, shutil, time

YAML = os.environ.get("ENVYAML",
                      "/home/data/groot/MIGRATE/ENV/config/environment_groot.yaml")
ENVDIR = os.environ.get("ENVDIR", "/home/data/groot/MIGRATE/ENV")
RUNNER = os.environ.get("RUNNER", "/home/data/groot/work/run_groot31.py")
NOMINAL = (-0.045, -0.100)          # 형 원본 캔 배치
RESULT_RE = re.compile(r"^RESULT\s+(.*)$", re.M)
# --- patch31: video+input-dump ---
VIDROOT = os.environ.get("VIDROOT", "")
VIDFPS = os.environ.get("VIDFPS", "20")


def encode(vdir, stem):
    """png 시퀀스를 mp4 로 묶는다. ffmpeg 이 없으면 png 를 그대로 남긴다."""
    pngs = sorted(glob.glob(os.path.join(vdir, "f*.png")))
    if not pngs:
        return ""
    mp4 = os.path.join(os.path.dirname(vdir.rstrip("/")), stem + ".mp4")
    try:
        r = subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-framerate", VIDFPS,
             "-i", os.path.join(vdir, "f%05d.png"),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", mp4],
            capture_output=True, timeout=300)
        if r.returncode == 0 and os.path.exists(mp4):
            for p in pngs:
                os.remove(p)
            return mp4
        print("[pos_grid] ffmpeg rc=%s -> png 유지 (%s)" % (r.returncode, vdir))
    except FileNotFoundError:
        print("[pos_grid] ffmpeg 없음 -> png 유지:", vdir)
    except Exception as e:
        print("[pos_grid] 인코딩 실패(%s) -> png 유지: %s" % (e, vdir))
    return vdir


def patch_yaml(path, x, y):
    s = open(path).read()
    i = s.index("prim_path: /World/Can")
    j = s.index("translate_xy:", i)
    k = s.index("orientation_rpy_deg:", j)
    s = s[:j] + "translate_xy:\n    - %.4f\n    - %.4f\n    " % (x, y) + s[k:]
    open(path, "w").write(s)


def parse_result(text):
    m = RESULT_RE.findall(text)
    if not m:
        return None
    out = {}
    for tok in m[-1].split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dx", default="0")
    ap.add_argument("--dy", default="0")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--tag", default="grid")
    ap.add_argument("--out", default="/home/data/groot/work/pos_grid.csv")
    ap.add_argument("--logdir", default="/home/data/groot/work/gridlogs")
    ap.add_argument("--timeout", type=int, default=900)
    ap.add_argument("--dry", action="store_true", help="yaml 패치만 하고 실행은 생략")
    a = ap.parse_args()

    dxs = [float(v) for v in a.dx.split(",")]
    dys = [float(v) for v in a.dy.split(",")]
    os.makedirs(a.logdir, exist_ok=True)

    bak = a.out + ".yamlbak"
    shutil.copyfile(YAML, bak)
    print("[pos_grid] yaml backed up ->", bak)

    def restore(*_):
        try:
            shutil.copyfile(bak, YAML)
            print("\n[pos_grid] yaml restored from backup")
        except Exception as e:
            print("[pos_grid] RESTORE FAILED:", e)
    signal.signal(signal.SIGINT, lambda *x: (restore(), sys.exit(130)))
    signal.signal(signal.SIGTERM, lambda *x: (restore(), sys.exit(143)))

    rows = []
    try:
        f = open(a.out, "w", newline="")
        wr = csv.writer(f)
        wr.writerow(["tag", "dx", "dy", "rep", "ok", "fit", "close_step",
                     "err_dx", "err_dy", "err_dz", "lift", "abort", "nstep",
                     "canx", "cany", "clampx", "clampy", "nclose", "last_dy",
                     "cammode", "frontpos", "log", "vid",
                     "det_dx", "det_dy", "det_dz", "anchor"])
        for dy in dys:
            for dx in dxs:
                patch_yaml(YAML, NOMINAL[0] + dx, NOMINAL[1] + dy)
                for rep in range(1, a.reps + 1):
                    log = os.path.join(
                        a.logdir, "%s_dx%+.3f_dy%+.3f_r%d.log" % (a.tag, dx, dy, rep))
                    if a.dry:
                        print("[dry] would run", log)
                        continue
                    t0 = time.time()
                    # # --- patch31: video+input-dump ---  실행마다 고유 VIDDIR. 서로 덮어쓰지 않는다.
                    env = os.environ.copy()
                    stem = "%s_dx%+.3f_dy%+.3f_r%d" % (a.tag, dx, dy, rep)
                    vdir = os.path.join(VIDROOT, stem) if VIDROOT else ""
                    if vdir:
                        env["VIDDIR"] = vdir
                    with open(log, "w") as lf:
                        try:
                            p = subprocess.run(
                                [sys.executable, "-u", RUNNER,
                                 "--config", "config/environment_groot.yaml"],
                                cwd=ENVDIR, stdout=lf, stderr=subprocess.STDOUT,
                                timeout=a.timeout, env=env)
                            rc = p.returncode
                        except subprocess.TimeoutExpired:
                            rc = -9
                            lf.write("\n[pos_grid] TIMEOUT\n")
                    vid = encode(vdir, stem) if vdir else ""
                    r = parse_result(open(log, errors="ignore").read()) or {}
                    if int(r.get("close_step", -1)) < 0:
                        # CLOSE 가 없었으면 9.99 는 오차가 아니다. 평균에 섞이면 지표가 죽는다.
                        r["dx"] = r["dy"] = r["dz"] = ""
                    row = [a.tag, dx, dy, rep,
                           int(r.get("ok", 0)), int(r.get("fit", 0)),
                           int(r.get("close_step", -1)),
                           r.get("dx", ""), r.get("dy", ""), r.get("dz", ""),
                           r.get("lift", ""), int(r.get("abort", 0)),
                           int(r.get("nstep", 0)),
                           r.get("canx", ""), r.get("cany", ""),
                           r.get("clampx", ""), r.get("clampy", ""),
                           r.get("nclose", ""), r.get("last_dy", ""),
                           r.get("cammode", ""), r.get("frontpos", ""), log, vid,
                           r.get("detx", ""), r.get("dety", ""),
                           r.get("detz", ""), r.get("anchor", "")]  # # --- patch31_det: ANCHOR=detect ---
                    wr.writerow(row)
                    f.flush()
                    rows.append(row)
                    print("[%s] dx%+.3f dy%+.3f r%d  ok=%d fit=%d dy_err=%s  %.0fs rc=%s"
                          % (a.tag, dx, dy, rep, row[4], row[5], row[8],
                             time.time() - t0, rc))
        f.close()
    finally:
        restore()

    # ---- 요약 매트릭스 ----
    def cell(dx, dy, key):
        v = [r for r in rows if abs(r[1] - dx) < 1e-9 and abs(r[2] - dy) < 1e-9]
        if not v:
            return None
        if key == "ok":
            return sum(r[4] for r in v), len(v)
        vals = [abs(float(r[8])) for r in v if r[8] not in ("", None)]
        return (sum(vals) / len(vals)) if vals else None

    print("\n== 성공률 (행 dy / 열 dx) ==")
    print("       " + "".join("%9.3f" % d for d in dxs))
    for dy in dys:
        line = "%7.3f" % dy
        for dx in dxs:
            c = cell(dx, dy, "ok")
            line += "%9s" % ("%d/%d" % c if c else "-")
        print(line)

    print("\n== 닫는 순간 |dy| 평균 [m]  (허용 0.0063) ==")
    print("       " + "".join("%9.3f" % d for d in dxs))
    for dy in dys:
        line = "%7.3f" % dy
        for dx in dxs:
            c = cell(dx, dy, "dy")
            line += "%9s" % ("%.4f" % c if c is not None else "-")
        print(line)

    tot = sum(r[4] for r in rows)
    print("\nTOTAL %d / %d" % (tot, len(rows)))
    print("CSV:", a.out)


if __name__ == "__main__":
    main()
