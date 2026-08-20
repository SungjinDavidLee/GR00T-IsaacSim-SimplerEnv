#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
obj_detect.py v2 -- det_server 에 텍스트를 보내 물체 마스크를 받는다.

 v1 문제: 서버가 박스 0개를 주면 조용히 HSV 로 폴백해서, 프롬프트를 뭘 줘도
          캔만 잡았다. **언어가 작동하는지 알 수 없는 상태였다.**
 v2 수정:
   · 서버는 문턱 없이 상위 k개를 준다. 판정은 여기서 한다.
   · 상위 3개 점수를 항상 출력한다.
   · DETMINSCORE 미만이면 **명시적으로 실패를 알린다.** 조용히 HSV 로 넘어가지 않는다.
     (폴백을 원하면 DETFALLBACK=1 — 그때도 로그에 폴백했다고 찍힌다)

 env:
   DETADDR      tcp://127.0.0.1:5556
   DETPROMPT    "a red soda can"     ; 쉼표로 여러 후보
   DETMINSCORE  0.03                 ; 이보다 낮으면 실패 처리
   DETMAXFRAC   0.05                 ; 이미지 면적의 이 비율보다 큰 박스는 버린다
   DETMINFRAC   0.0005               ; 너무 작은 박스도 버린다
   DETSHRINK    0.80                 ; 박스 축소율 (경계 depth 오염 방지)
   DETTIMEOUT   20000                ; ms. 첫 호출은 모델 로딩으로 느릴 수 있다
"""
import json
import os

import numpy as np

_sock = None


def _connect():
    global _sock
    if _sock is not None:
        return _sock
    import zmq
    addr = os.environ.get("DETADDR", "tcp://127.0.0.1:5556")
    ctx = zmq.Context.instance()
    s = ctx.socket(zmq.REQ)
    t = int(os.environ.get("DETTIMEOUT", "20000"))
    s.setsockopt(zmq.RCVTIMEO, t)
    s.setsockopt(zmq.SNDTIMEO, t)
    s.setsockopt(zmq.LINGER, 0)
    s.connect(addr)
    print("[obj_detect] connected", addr)
    _sock = s
    return s


def detect(rgb, prompt=None):
    """(box, score, label) 목록, 점수 내림차순. 통신 실패면 빈 리스트."""
    global _sock
    prompt = prompt or os.environ.get("DETPROMPT", "a red soda can")
    texts = [t.strip() for t in prompt.split(",") if t.strip()]
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    hdr = {"shape": list(rgb.shape), "dtype": "uint8", "texts": texts}
    try:
        s = _connect()
        s.send_multipart([json.dumps(hdr).encode(), rgb.tobytes()])
        rep = json.loads(s.recv_string())
    except Exception as e:
        print("[obj_detect] 서버 통신 실패:", e)
        print("   det_server.py 가 떠 있는지 확인:  ss -tlnp | grep 5556")
        _sock = None
        return []
    if rep.get("error"):
        print("[obj_detect] 서버 오류:", rep["error"])
    return list(zip(rep.get("boxes", []), rep.get("scores", []),
                    rep.get("labels", [])))


def mask_from_text(rgb, prompt=None):
    """can_detect.red_mask 와 같은 규약: (mask uint8 HxW, area) 또는 (None, 0)."""
    dets = detect(rgb, prompt)
    if not dets:
        print("[obj_detect] 박스 0개 — 검출 실패")
        return None, 0
    print("[obj_detect] top3: " + " | ".join(
        "%s %.4f" % (l, s) for _, s, l in dets[:3]))
    # 크기 필터: 테이블/서랍 전체 같은 거대 박스를 버린다.
    # 실측(c2_rgb.png): 캔 52x76 px = 이미지의 0.4 %, 오검출은 815x174 = 15 %
    H, W = rgb.shape[:2]
    lo = float(os.environ.get("DETMINFRAC", "0.0005"))
    hi = float(os.environ.get("DETMAXFRAC", "0.05"))
    keep = []
    for (x0, y0, x1, y1), sc, lb in dets:
        f = abs((x1 - x0) * (y1 - y0)) / float(H * W)
        if lo <= f <= hi:
            keep.append(((x0, y0, x1, y1), sc, lb))
        else:
            print("[obj_detect] 크기로 기각 %s %.4f  면적비 %.4f" % (lb, sc, f))
    if not keep:
        print("[obj_detect] 크기 필터 통과 박스 없음"); return None, 0
    dets = keep
    (x0, y0, x1, y1), score, label = dets[0]
    minsc = float(os.environ.get("DETMINSCORE", "0.03"))
    if score < minsc:
        print("[obj_detect] 최고 점수 %.4f < DETMINSCORE %.3f — 실패 처리" % (score, minsc))
        print("   모델을 바꿔볼 것: DETMODEL=google/owlv2-base-patch16-ensemble")
        return None, 0
    h, w = rgb.shape[:2]
    sh = float(os.environ.get("DETSHRINK", "0.80"))
    cx, cy = (x0 + x1) * 0.5, (y0 + y1) * 0.5
    bw, bh = (x1 - x0) * sh * 0.5, (y1 - y0) * sh * 0.5
    a = max(0, int(round(cx - bw))); b = min(w, int(round(cx + bw)))
    c = max(0, int(round(cy - bh))); d = min(h, int(round(cy + bh)))
    if b <= a or d <= c:
        print("[obj_detect] 박스가 비었다"); return None, 0
    m = np.zeros((h, w), np.uint8)
    m[c:d, a:b] = 1
    area = int((b - a) * (d - c))
    print("[obj_detect] 채택 '%s' score %.4f box [%.0f %.0f %.0f %.0f] area %d"
          % (label, score, x0, y0, x1, y1, area))
    return m, area
