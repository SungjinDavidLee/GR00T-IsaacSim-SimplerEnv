#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch31_det2.py -- run_groot31.py 에 POLICYCHK 를 추가한다.

 왜 지금 이걸 먼저 해야 하는가:
   동일 설정 재현이 안 된다.
     t_gh48  (NOREOPEN=0, GHOLD=48, CLOSEMIN=4)  ->  1/3
     u_off48 (완전히 같은 설정)                   ->  0/4
   조건을 바꾸지 않았는데 결과가 다르다. 이 상태에서 3~4판으로 조건을 비교하면
   **잡음을 신호로 읽는다.** 어젯밤부터 그러고 있었다.

 변동의 출처 후보:
   (a) 정책이 확률적이다  — GR00T N1.7 의 action head 는 flow matching 계열이라
       호출마다 노이즈를 새로 뽑으면 같은 관측에도 다른 행동이 나온다  ← 가장 유력
   (b) Isaac Sim 물리/렌더가 비결정적이다
   (c) 검출이 흔들린다 — 이미 기각. detx/dety 변동 2 mm 이내

 POLICYCHK=N 이면 step 0 에서 **완전히 같은 관측**으로 정책을 N번 호출하고
 반환된 청크의 차이를 출력한다. (a)인지 아닌지 한 판으로 판정된다.

 사용: python3 patch31_det2.py [work_dir]
"""
import os, sys, py_compile, shutil

WORK = sys.argv[1] if len(sys.argv) > 1 else "/home/data/groot/work"
MARK = "# --- patch31_det2: POLICYCHK ---"

OLD = """        act = client.get_action(gobs)"""
NEW = """        """ + MARK + """
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
        act = client.get_action(gobs)"""


def main():
    p = os.path.join(WORK, "run_groot31.py")
    s = open(p).read()
    if MARK in s:
        print("[patch31_det2] 이미 패치됨 — 건너뜀"); return
    n = s.count(OLD)
    if n != 1:
        raise SystemExit("[patch31_det2] 앵커 %d 곳 (1이어야 함). 중단." % n)
    s = s.replace(OLD, NEW)
    shutil.copyfile(p, p + ".bak_pk")
    open(p, "w").write(s)
    py_compile.compile(p, doraise=True)
    print("[patch31_det2] ok  (백업: %s.bak_pk)" % p)


if __name__ == "__main__":
    main()
