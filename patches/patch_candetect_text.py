#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_candetect_text.py -- can_detect.py 에 DETMODE=text 경로를 추가한다.

  DETMODE=hsv   (기본) 지금까지와 동일. 빨강 HSV 마스킹
  DETMODE=text  det_server(OWL-ViT)에 DETPROMPT 를 보내 물체를 지목

 depth 역투영·반지름 보정·좌표 변환은 **그대로 재사용**한다. 마스크 출처만 바뀐다.
 사용: python3 patch_candetect_text.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch_candetect_text: DETMODE ---"

OLD = """    mask, area = red_mask(rgb)"""
NEW = """    """ + MARK + """
    if os.environ.get("DETMODE", "hsv").startswith("text"):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import obj_detect
        mask, area = obj_detect.mask_from_text(rgb)
        if mask is None and int(os.environ.get("DETFALLBACK", "0")):
            print("[can_detect] 텍스트 검출 실패 -> HSV 로 폴백")
            mask, area = red_mask(rgb)
    else:
        mask, area = red_mask(rgb)"""


def main():
    p = os.path.join(WORK, "can_detect.py")
    s = open(p).read()
    if MARK in s:
        print("[patch_candetect_text] 이미 패치됨"); return
    n = s.count(OLD)
    if n != 1:
        raise SystemExit("앵커 %d 곳 (1이어야 함). 중단." % n)
    s = s.replace(OLD, NEW)
    if "\nimport sys" not in s and "import sys" not in s.split("\n\n")[0]:
        s = s.replace("import os\n", "import os\nimport sys\n", 1)
    shutil.copyfile(p, p + ".bak_text")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("[patch_candetect_text] ok  (백업: %s.bak_text)" % p)


if __name__ == "__main__":
    main()
