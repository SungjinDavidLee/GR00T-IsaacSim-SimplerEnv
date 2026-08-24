# 파일 구성

`<work>`는 실행 디렉토리를 가리킨다.

---

## 1. 핵심 — 이것이 없으면 동작하지 않는다

| 파일 | 역할 |
|---|---|
| `src/run_groot31.py` | 메인 러너. 시뮬레이터와 정책 서버 연동, 홈잉, 정책 호출, IK, 파지 판정, 로깅 |
| `src/can_detect.py` | wrist RGB-D 물체 위치 추정. 시뮬레이터 좌표 조회를 대체한다 |
| `src/obj_detect.py` | 검출 서버 클라이언트. 텍스트 → 박스 → 마스크 |
| `src/det_server.py` | 자연어 검출 서버 (ZMQ `:5556`) |
| `src/pos_grid.py` | 위치 스윕 하네스. 씬 설정 백업·복원, 결과 파싱, CSV, 영상 인코딩 |
| `scripts/srv.sh` | 정책 서버 기동 (ZMQ `:5555`) |
| `scripts/pick3.sh` | 물체별 파지 실행 |
| `scripts/front_mode.py` | 보조 카메라 배치 전환. 모든 실험 전에 기준 배치로 고정한다 |

---

## 2. 실험 스크립트

| 파일 | 역할 |
|---|---|
| `scripts/abc_eval2.sh` | 절제 실험 실행 및 집계 호출 |
| `tools/abc_report.py` | 절제 실험 집계. Wilson 구간, Fisher 정확검정, 차이의 구간, 오류 코드 교차표 |
| `tools/armdiag.py` | 조건별 종료 방식 진단. 스텝 수, 전도, 파지 명령 유무 |
| `scripts/geomgate.sh` | 검출 방식 비교 게이트. 격자 5곳, 파지 시도 없음 |
| `scripts/rotgate.sh` | 회전 3-DoF 게이트 및 A/B |
| `tools/polat.py` | 정책 호출 지연 측정. 시뮬레이터를 띄우지 않는다 |
| `tools/ksweep_report.py` | K 스윕 집계. 성공률과 비용을 한 표로 합친다 |
| `tools/errclass.py` | 판별 로그의 오류 코드 자동 분류 |
| `tools/rep3.py` | 결과 분석 |
| `tools/gen_figures.py` | 문서용 SVG 그림 생성 |

---

## 3. 패치

러너와 검출 모듈에 순차 적용된 스크립트다. 각 패치는 앵커가 정확히 한 곳일 때만
동작하고, 원본을 백업하며, 두 번 실행해도 안전하다.

| 파일 | 추가한 것 |
|---|---|
| `patches/patch31_det.py` | 검출 좌표 기반 앵커 |
| `patches/patch31_cam.py` | 카메라 오프셋, 판정에 dx/dz 포함 |
| `patches/patch31_ksamp.py` | K회 호출 후 중앙값 |
| `patches/patch31_obj.py` | 평가 대상 물체 선택 |
| `patches/patch31_zero.py` | 정책 출력 0 대조 실험 |
| `patches/patch31_place.py` 외 | 놓기 단계 |
| `patches/patch31_policyik4.py` | 스크립트 IK 정책 (절제 실험의 C·C0 조건) |
| `patches/patch_candetect_text.py` | 자연어 검출 사용 |
| `patches/patch_candetect_aim.py` | 조준 보정 |
| `patches/patch_candetect_geom2.py` | 물체 상수 제거 |
| `patches/patch_geomaim.py` | 상수 제거 분기에서도 조준 보정 적용 |

### 적용 시 주의

패치는 **줄 단위로 앵커를 찾아 해당 줄의 실제 들여쓰기에 맞춰 삽입**해야 한다.
문자열 치환은 부분문자열 매칭이므로, 4칸 들여쓰기 패턴이 8칸 줄 안에 포함되어
잘못 매칭된다. 초기 패치에서 이 문제로 파일이 손상된 사례가 있다.

또한 조기 `return`이 있는 분기를 추가할 때는 **그 분기가 건너뛰는 코드**를 반드시
확인해야 한다. 상수 제거 분기가 조준 보정 코드를 건너뛰어, 일부 환경변수만
반영되는 상태로 16판이 실행된 사례가 있다.

---

## 4. 산출물

| 경로 | 내용 |
|---|---|
| `<work>/gridlogs_abc/` | 절제 실험 판별 로그 |
| `<work>/gridlogs_abctop/` | 상수 제거 검출로 실행한 조건 B |
| `<work>/gridlogs_abck1`, `k4`, `k16` | K 스윕 |
| `<work>/gridlogs_geom/` | 검출 방식 비교 |
| `<work>/gridlogs_rot/` | 회전 3-DoF 게이트 |
| `<work>/vid_abc/`, `vid_rot/` | 실행 영상 |
| `<work>/exp_*.csv` | 판별 CSV |

로그 파일명 규칙은 `<태그>_dx±0.000_dy±0.000_r<반복>.log`이며,
대응하는 영상은 확장자만 다르다.

---

## 5. 저장소에 포함하지 않는 것

| 항목 | 이유 |
|---|---|
| 씬 정의 파일 | 이 저장소의 산출물이 아니다 |
| 체크포인트 | 용량. 내려받기 명령만 기재 |
| 로그·영상 전체 | 용량. 대표 자료만 |
