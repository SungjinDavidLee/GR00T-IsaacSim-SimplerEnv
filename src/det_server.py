#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
det_server.py v2 -- 자연어로 물체를 지목하는 검출 서버.

 v1 문제: threshold 0.05 로 걸러서 **박스가 0개** 나왔고, 그래서 HSV 로 폴백 →
          프롬프트를 뭘 줘도 캔만 잡았다.
 v2 수정:
   · 문턱을 0 으로 두고 **항상 상위 k개를 반환**한다. 판정은 클라이언트가 한다.
   · 상위 3개 점수를 로그에 찍어 "정말 못 찾는지 / 문턱 문제인지" 보이게 한다.
   · AutoModelForZeroShotObjectDetection 을 써서 owlvit / owlv2 / grounding-dino
     아무거나 DETMODEL 로 바꿔 끼울 수 있다.
   · **오프라인 테스트 모드** — 시뮬 없이 저장된 PNG 로 바로 확인한다.

 작은 물체에 약하면 모델부터 바꿀 것 (owlv2 가 훨씬 낫다):
   DETMODEL=google/owlv2-base-patch16-ensemble

 오프라인 확인 (서버 안 띄우고, 1분):
   python det_server.py --test /home/data/groot/work/probe/c2_rgb.png \
       "a red soda can" "a green sponge" "a plastic water bottle"

 서버 실행 (GR00T venv):
   cd /home/data/groot/Isaac-GR00T && uv run python /home/data/groot/work/det_server.py
"""
import json
import os
import sys

import numpy as np

PORT = int(os.environ.get("DETPORT", "5556"))
MODEL = os.environ.get("DETMODEL", "google/owlvit-base-patch32")
TOPK = int(os.environ.get("DETTOPK", "10"))


def load():
    import torch
    from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print("[det_server] loading %s on %s ..." % (MODEL, dev), flush=True)
    proc = AutoProcessor.from_pretrained(MODEL)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL).to(dev).eval()
    return torch, proc, model, dev


def infer(torch, proc, model, dev, img, texts):
    """문턱 없이 상위 TOPK 를 돌려준다."""
    h, w = img.shape[:2]
    with torch.no_grad():
        inp = proc(text=[texts], images=img, return_tensors="pt").to(dev)
        out = model(**inp)
        ts = torch.tensor([[h, w]]).to(dev)
        try:
            res = proc.post_process_grounded_object_detection(
                out, threshold=0.0, target_sizes=ts)[0]
        except (AttributeError, TypeError):
            res = proc.post_process_object_detection(
                out, threshold=0.0, target_sizes=ts)[0]
    boxes = res["boxes"].detach().cpu().numpy()
    scores = res["scores"].detach().cpu().numpy()
    lab = res["labels"].detach().cpu().numpy()
    if scores.size == 0:
        return [], [], []
    order = np.argsort(scores)[::-1][:TOPK]
    labs = []
    for i in order:
        li = int(lab[i])
        labs.append(texts[li] if 0 <= li < len(texts) else str(li))
    return boxes[order].tolist(), scores[order].tolist(), labs


def test_mode(argv):
    import cv2
    if not argv:
        sys.exit("사용: --test <image.png> [\"prompt\" ...]")
    path = argv[0]
    texts = argv[1:] or ["a red soda can"]
    img = cv2.imread(path)
    if img is None:
        sys.exit("이미지를 못 읽음: " + path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    print("이미지 %s  %s" % (path, img.shape))
    torch, proc, model, dev = load()
    boxes, scores, labs = infer(torch, proc, model, dev, img, texts)
    if not boxes:
        print("박스 0개 — 모델이 아무것도 못 찾았다.")
        return 1
    print("\n%-30s %8s  box" % ("label", "score"))
    for b, s, l in list(zip(boxes, scores, labs))[:8]:
        print("%-30s %8.4f  [%.0f %.0f %.0f %.0f]" % (l, s, b[0], b[1], b[2], b[3]))
    vis = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    for b, s, l in list(zip(boxes, scores, labs))[:3]:
        cv2.rectangle(vis, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])),
                      (0, 255, 0), 2)
        cv2.putText(vis, "%s %.3f" % (l, s), (int(b[0]), max(14, int(b[1]) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    outp = os.path.join(os.path.dirname(path) or ".", "det_test.png")
    cv2.imwrite(outp, vis)
    print("\n시각화: %s" % outp)
    print("상위 점수가 0.1 미만이면 모델을 바꿔볼 것:")
    print("  DETMODEL=google/owlv2-base-patch16-ensemble python det_server.py --test ...")
    return 0


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        return test_mode(sys.argv[2:])
    import zmq
    torch, proc, model, dev = load()
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind("tcp://*:%d" % PORT)
    print("[det_server] ready on tcp://*:%d  (model=%s)" % (PORT, MODEL), flush=True)
    while True:
        try:
            parts = sock.recv_multipart()
            hdr = json.loads(parts[0].decode())
            h, w, c = hdr["shape"]
            img = np.frombuffer(parts[1], dtype=np.dtype(hdr["dtype"])).reshape(h, w, c)
            texts = hdr.get("texts") or ["a red soda can"]
            boxes, scores, labs = infer(torch, proc, model, dev, img, texts)
            top = ", ".join("%s %.3f" % (l, s)
                            for l, s in list(zip(labs, scores))[:3]) or "없음"
            print("[det_server] %s -> %d boxes | top3: %s"
                  % (texts, len(boxes), top), flush=True)
            sock.send_string(json.dumps({"boxes": boxes, "scores": scores,
                                         "labels": labs}))
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("[det_server] error:", e, flush=True)
            try:
                sock.send_string(json.dumps({"boxes": [], "scores": [],
                                             "labels": [], "error": str(e)}))
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
