# CODEMAP — 전체 파일 목록·위치·용도·상태

모든 파일은 워크스테이션의 `/home/data/groot/work/` 에 있습니다(명시된 것 제외).
**상태** 열의 의미:

- `핵심` — 시스템 동작에 필요. GitHub 에 반드시 올릴 것
- `진단` — 원인 규명에 쓴 도구. 재현·검증용으로 올릴 가치 있음
- `패치` — `run_groot31.py` / `can_detect.py` 를 순차 수정한 스크립트. 이력 보존용
- `폐기` — 잘못됐거나 대체됨. **올리지 말거나 `deprecated/` 로 분리**

---

## 1. 핵심 — 이게 없으면 안 돌아갑니다

| 파일 | 용도 | 상태 |
|---|---|---|
| `run_groot31.py` | 메인 러너. Isaac Sim ↔ GR00T 서버 연동, 홈잉, 정책 호출, IK, 파지 판정, 로깅 | 핵심 |
| `can_detect.py` | **wrist RGB-D 물체 위치 추정.** 시뮬레이터 좌표 조회를 대체한 모듈 | 핵심 |
| `obj_detect.py` | 검출 서버 클라이언트. 텍스트 → 박스 → 마스크 | 핵심 |
| `det_server.py` | **OWL-v2 자연어 검출 서버** (ZMQ `:5556`). GR00T venv 에서 실행 | 핵심 |
| `pick3.sh` | 물체별 파지 실행. 상수·프롬프트·지시문이 한 곳에 묶임 | 핵심 |
| `pos_grid.py` | 위치 스윕 하네스. yaml 백업·복원, RESULT 파싱, CSV, 영상 인코딩 | 핵심 |
| `rep3.py` | 결과 분석. 태그별 요약 + 성공/실패 판 비교 + `fit=1 ok=0` 격차 추출 | 핵심 |
| `front_mode.py` | front 카메라 배치 전환. **모든 실험 앞에 `original` 로 고정**(설정 표류 방지) | 핵심 |
| `srv.sh` | GR00T 정책 서버 기동 (`:5555`) | 핵심 |
| `randtest.sh` + `pos_grid_one.sh` | 무작위 위치 시험. ±60 mm 임의 좌표 N곳 | 핵심 |

### 저장소 외부 의존 (포함하지 않음, 수정하지 않음)

| 경로 | 설명 |
|---|---|
| `/home/data/groot/MIGRATE/ENV/isaac_simpler_env_multi_object.py` | 연구실 선행 연구자가 만든 Isaac Sim 씬 |
| `/home/data/groot/MIGRATE/ENV/config/environment_groot.yaml` | 씬 설정(물체 배치, 카메라). 실행 중 임시 수정 후 **반드시 복원** |
| `/home/data/groot/checkpoints/GR00T-N1.7-LIBERO/` | NVIDIA 체크포인트 61 GB |

---

## 2. 패치 — `run_groot31.py` 를 순서대로 고친 스크립트

**적용 순서가 중요합니다.** 각 패치는 앵커 문자열이 정확히 1곳일 때만 동작하고,
원본을 `.bak_*` 로 백업하며, 두 번 실행해도 안전합니다.

| 순서 | 파일 | 추가한 것 | 왜 |
|---|---|---|---|
| 1 | `patch31.py` | 영상 기록 4분할, **모델 입력 덤프**, 첫 파지만 기록 | 재시도 값이 첫 파지를 덮어써 지표가 오염됐음 |
| 2 | `patch31_det.py` | **`ANCHOR=detect`** — 시뮬 좌표 대신 카메라 추정 사용 | 이 프로젝트의 핵심 변경 |
| 3 | `patch31_cam.py` | `CAMSHIFT`, **`fit` 판정에 dx/dz 포함** | `fit` 이 dy 만 봐서 가짜 성공이 찍혔음 |
| 4 | `patch31_latch.py` | `NOREOPEN` — 첫 파지 후 열기 명령 무시 | 물었다 놓는 극한주기가 성공을 갉아먹음 |
| 5 | `patch31_det2.py` | `POLICYCHK` — 정책 결정성 검사 | 같은 설정이 재현되지 않는 원인 규명용 |
| 6 | `patch31_ksamp.py` | **`KSAMP`/`KSAMPOPEN`** — K샘플 중앙값 | 정책이 확률적임을 확인한 뒤 대응 |
| 7 | `patch31_obj.py` | `OBJ` — 평가 대상 물체 선택 | `find_obj(backend,"can")` 이 5곳 하드코딩돼 있었음 |
| 8 | `patch31_zero.py` | `ZEROACT` — 정책 출력 0/무작위 대조 실험 | "정말 GR00T 가 하는가" 증명용 |

### `can_detect.py` 패치

| 파일 | 추가한 것 | 상태 |
|---|---|---|
| `patch_candetect_text.py` | `DETMODE=text` — HSV 대신 OWL-v2 사용 | 적용됨 |
| `patch_candetect_aim.py` | `AIMX/AIMY/AIMZ` — 베이스 프레임 조준 보정 | 적용됨 |
| `patch_candetect_geom.py` | `DETGEOM=top` — **물체 상수 제거** | **미적용·미검증** |

> **정리 제안:** 패치를 전부 적용한 최종 `run_groot31.py` / `can_detect.py` 를 올리고,
> 패치 스크립트는 `patches/` 폴더에 이력으로 보존하는 편이 읽기 쉽습니다.

---

## 3. 진단 도구 — 원인 규명에 쓴 것들

| 파일 | 무엇을 밝혔나 |
|---|---|
| `probe_can.py` | 검출 파이프라인 첫 측정. **여기서 오차 112 mm 를 발견** |
| `probe_can2.py` | **D405 실제 intrinsics(fx=469.27) 확정.** 축 규약, `up`≡`wrist` 동일성 확인 |
| `cam_rel.py` | 카메라가 `panda_hand` 기준 `[0.05, 0, 0]` 에 장착됨을 측정 |
| `check5.sh` | **정책이 확률적임을 확인**(`policy` 모드). 재현성·기준선 측정 |

---

## 4. 실험 스크립트 — 시행 이력

시간 순서대로 남긴 것입니다. 대부분 특정 가설을 검증하고 역할이 끝났습니다.

| 파일 | 검증한 가설 | 결과 | 상태 |
|---|---|---|---|
| `exp_fix.sh` v3 | front 배치, NSUBOPEN, 클램프 등 | 전 분기에 `front_pin` 추가(설정 표류 차단) | 진단 |
| `exp_det.sh` | `ANCHOR=detect` 도입, 검출 격자 교정 | 검출 오차 격자 전역 2 mm 확인 | 진단 |
| `grasp_now.sh` | `CAMSHIFT` 가 유효 손잡이인지 | 확인. 첫 `ok=1` | 진단 |
| `grasp2.sh` | `NSUBOPEN` 8/4/2 | dx 산포 46→12 mm | 진단 |
| `tune3.sh` | `CLOSEMIN`/`GHOLD`/`GDEB`/`ASCALE` | 표본 부족으로 판정 불가 | 폐기 |
| `tune4.sh` | `NOREOPEN` A/B | 표본 부족 | 폐기 |
| `tune5.sh` | **K=8 중앙값** | 조준 30 %→93 % | 진단 |
| `tune6.sh` | `CAMSHIFT` x 스윕 | **−0.0205 부터 절벽** 발견 | 진단 |
| `tune7.sh` | `FLOORB` 양수, x 미세 | `FLOORB` 양수는 악화 | 진단 |
| `tune8.sh` | `CAMSHIFT` z, `ROT6` | 미실행 | 폐기 |
| `run_all.sh` | geom / rot / pick3 통합 | `rot` 은 검출 오차 상태에서 측정 → 무효 | 진단 |
| `obj_run.sh` | 물체별 실행 | `pick3.sh` 로 대체 | 폐기 |
| `bias_fit.py` | `XBIAS/YBIAS` 보정 | **효과 없음(기울기 0.004) 확인 후 역할 종료** | 폐기 |
| (미작성) | **GR00T-only vs 외부 grounding 대조** | **아직 없음. README §8-9 / §9-1** | 필요 |
| `lang_test.sh` | 언어 사용 여부 정량화 | **stdin 결함으로 지시문 잘림. 결과 폐기** | 폐기 |
| `floor.sh` | `FLOORB` 스윕 | 병목 아님 확인 | 폐기 |

---

## 5. 이전 작업물 (이 프로젝트 이전부터 있던 것)

| 파일 | 용도 |
|---|---|
| `run_groot30.py` | 이전 러너. **수정하지 않고 보존** |
| `task1.py` | 스크립트 IK 기준선. GR00T 없이 IK 만으로 파지 |
| `report_pos.py` | 초기 리포트 도구. `rep3.py` 로 대체 |

---

## 6. 산출물 디렉토리

| 경로 | 내용 |
|---|---|
| `gridlogs/` | 판별 로그. `<tag>_dx±0.000_dy±0.000_r<N>.log` |
| `vid/` | 실행 영상 mp4. 4분할(원본 front/wrist + **모델 입력 224 두 장**) |
| `exp_*.csv` | 판별 CSV |
| `probe/` | 검출 진단 이미지 |

**영상의 아래 두 패널이 중요합니다.** 원본이 아니라 **정책에 실제로 들어간 224 이미지**라,
`CAMMODE` 마스킹이 걸렸는지 영상만 보고 확인할 수 있습니다.

---

## 7. GitHub 업로드 권장 구성

```
.
├── README.md                     ← 프로젝트 설명·결과·한계
├── docs/
│   └── CODEMAP.md                ← 이 문서
│       (측정 사실·한계·다음 실험은 README §6~§9 에 통합)
├── src/
│   ├── run_groot31.py            ← 패치 전부 적용된 최종본
│   ├── can_detect.py             ← 패치 전부 적용된 최종본
│   ├── obj_detect.py
│   ├── det_server.py
│   ├── pos_grid.py
│   └── rep3.py
├── scripts/
│   ├── pick3.sh
│   ├── randtest.sh
│   ├── pos_grid_one.sh
│   ├── front_mode.py
│   └── srv.sh
├── patches/                      ← 적용 이력 보존
│   ├── patch31.py ... patch31_zero.py
│   └── patch_candetect_*.py
├── tools/                        ← 진단
│   ├── probe_can.py
│   ├── probe_can2.py
│   ├── cam_rel.py
│   └── check5.sh
└── deprecated/                   ← 역할이 끝났거나 결함이 있던 것
    ├── bias_fit.py
    ├── lang_test.sh
    └── tune3.sh ... tune8.sh
```

### 올리지 말아야 할 것

| 항목 | 이유 |
|---|---|
| `isaac_simpler_env_multi_object.py`, `environment_groot.yaml` | 연구실 선행 연구자 소유. 별도 협의 필요 |
| 체크포인트 61 GB | 용량. `hf download` 명령만 README 에 기재 |
| `gridlogs/`, `vid/` 전체 | 용량. 대표 로그·영상 몇 개만 |
| 과제 협약서·미공개 칩 스펙 | 기관 정책 |

### `deprecated/` 를 남기는 이유

지운 실험은 **왜 그 경로를 버렸는지**를 설명하지 못합니다.
`XBIAS/YBIAS` 를 213 mm 흔들어도 소용없었다는 기록,
`fit` 지표가 dy 만 봐서 가짜 성공이 나왔다는 기록이 남아 있어야
같은 함정에 다시 빠지지 않습니다. **README §5, §7 과 짝지어 읽히도록 두는 편이 낫습니다.**
