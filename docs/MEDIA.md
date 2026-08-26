# 영상·이미지 자료 배치

문서에서 참조하는 시각 자료의 출처와 저장 위치를 정리한다.
`<work>` 는 실행 디렉토리, `<repo>` 는 이 저장소의 최상위 경로를 가리킨다.

---

## 1. 이미 저장소에 포함된 그림

측정값으로 생성한 벡터 그림이다. 외부 의존이 없고 텍스트로 diff 가 된다.

| 파일 | 내용 | 참조 위치 |
|---|---|---|
| `docs/media/pipeline.svg` | 시스템 구성도 | README §1 |
| `docs/media/ablation.svg` | 절제 실험 성공률과 신뢰구간 | README §2 |
| `docs/media/ksweep.svg` | K별 성공률·비용·전도율 | README §3 |
| `docs/media/detection.svg` | 검출 방식별 격자 변동폭 | README §4 |
| `docs/media/place.svg` | 놓기 단계 — 이송 도달, 하강 없음 | README §5 |

수치를 수정할 경우 `tools/gen_figures.py` 를 고쳐 다시 생성한다.
그림의 숫자를 직접 편집하지 않는다.

---

## 2. 영상이 저장되는 규칙

실행 시 각 판이 mp4 로 저장된다. **로그 디렉토리와 영상 디렉토리가 짝을 이룬다.**

```
<work>/gridlogs_<접미어>/<태그>_dx±0.000_dy±0.000_r1.log
<work>/vid_<접미어>/<태그>_dx±0.000_dy±0.000_r1.mp4
```

확장자만 다르고 나머지는 같다. 현재 있는 디렉토리는 다음으로 확인한다.

```bash
cd <work>
ls -d gridlogs_* vid_*
```

| 접미어 | 실험 |
|---|---|
| `abc` | 절제 실험 A/B/C/C0 (검출 `ray`) |
| `abctop` | 조건 B, 물체 상수 제거 검출 |
| `abck1` · `abck4` · `abck16` | 추론 시점 샘플링 K 스윕 |
| `abcnoaim` | 조건 B, 조준 보정 0 |
| `geom` | 검출 방식 비교 (파지 시도 없음) |
| `rot` | 회전 3-DoF 게이트 |
| `pert` · `pertslow` · `pertflip` · `pertrnd` | 접근 중 물체 이동 |
| `place*` | 놓기 단계 (조건마다 접미어가 다름) |

> **주의.** 영상 경로에 접미어를 붙이는 수정은 실험 도중에 적용되었다.
> 그 이전에 실행한 조건들은 태그와 좌표가 같아 **`vid_abc/` 안에서 서로
> 덮어썼다.** 남아 있는 파일이 어느 조건인지는 파일 시각으로 판단해야 한다.
> 로그는 디렉토리가 분리되어 있어 영향이 없다.

---

## 3. 문서에 넣을 영상

화면은 4분할이며 **아래 두 장이 정책에 실제로 입력되는 224 × 224 영상**이다.
위 두 장은 원본 카메라 영상이다.

### 3-1. 절제 실험 — 실패의 성질이 다르다

| 저장소 경로 | 원본 | 보여주는 것 |
|---|---|---|
| `docs/media/topple.gif` | `<work>/vid_abc/abcB_09_dx-0.030_dy+0.059_r1.mp4` | 정책이 물체를 넘어뜨리는 실패 (12스텝 종료) |
| `docs/media/no_grasp.gif` | `<work>/vid_abc/abcC0_03_dx-0.060_dy-0.019_r1.mp4` | 스크립트 IK 가 문턱을 만족하지 못해 파지 명령을 내지 않는 실패 (90스텝 소진) |
| `docs/media/aim_offset.gif` | `<work>/vid_abc/abcC_01_dx+0.003_dy-0.045_r1.mp4` | 조준 보정 공유로 41 mm 옆을 무는 실패 |

앞의 두 편을 나란히 두는 것이 절제 실험의 요점이다. 두 조건은 성공률에서
구분되지 않지만 실패하는 방식이 다르다.

### 3-2. 놓기 단계 — 이송은 되고 하강은 안 된다

| 저장소 경로 | 원본 | 보여주는 것 |
|---|---|---|
| `docs/media/place_reach.gif` | `<work>/vid_placet1/plcgroot_03_dx+0.000_dy+0.000_r1.mp4` | 목표 11 mm 까지 이송하나 그리퍼를 열지 않음 |
| `docs/media/place_drop.gif` | `<work>/vid_placefin3/plcgroot_06_dx+0.000_dy+0.000_r1.mp4` | 정책이 s12 에 개방, 물체 전도 |
| `docs/media/place_high.gif` | `<work>/vid_placefin3/plcgroot_08_dx+0.000_dy+0.000_r1.mp4` | 0.55 m 상공에서 개방 |

### 3-3. 성공 판

성공 판의 파일명은 무작위 좌표에 따라 달라지므로 로그에서 찾는다.

```bash
cd <work>
grep -l "ok=1" gridlogs_abctop/abcB_*.log | head -3
```

출력된 로그 파일명에서 확장자만 바꾸면 대응하는 영상이 된다.

```
gridlogs_abctop/abcB_05_dx+0.012_dy-0.031_r1.log
   ->  vid_abctop/abcB_05_dx+0.012_dy-0.031_r1.mp4
```

이것을 `docs/media/pipeline.gif` 로 넣으면 README §1 의 구성도와 짝이 된다.

---

## 4. 변환

저장소에는 mp4 대신 짧은 GIF 를 넣는다. 용량이 작고 문서에서 바로 재생된다.

```bash
ffmpeg -i <원본>.mp4 -vf "fps=10,scale=480:-1" -loop 0 <repo>/docs/media/<이름>.gif
```

원본 mp4 전체는 저장소에 넣지 않는다. 대표 영상 몇 개만 넣고 나머지는 로그와
함께 별도 보관한다.

---

## 5. 캡션 규칙

영상에는 반드시 **조건, 판 번호, 좌표**를 캡션으로 적는다. 로그와 대조가
가능해야 영상이 근거로 기능한다.

```markdown
![물체 전도](docs/media/topple.gif)

*조건 B, 판 09, 물체 오프셋 dx −30 mm / dy +59 mm. 12스텝에서 물체 전도로 종료.*
```

```markdown
![놓기 하강 실패](docs/media/place_high.gif)

*놓기 단계, 정책 단독. 전환 s11 후 s23 에 개방. 개방 시 물체 높이 1.421 m
(테이블 0.865 m, 목표 0.941 m).*
```

---

## 6. 검출 진단 이미지

검출 결과 오버레이 이미지는 진단용 환경변수를 지정하면 저장된다.

| 원본 | 내용 |
|---|---|
| `<work>/probe/det.png` | 검출 마스크가 초록으로 겹쳐진 wrist 영상 |

물체 상수 사용/제거 두 방식의 오버레이를 나란히 넣으면 README §4 의 검출 비교를
보완한다. 저장소에는 `docs/media/detect_ray.png`, `docs/media/detect_top.png`
로 넣는다.

---

## 7. 저장소에 넣지 않는 것

| 항목 | 이유 |
|---|---|
| 체크포인트 | 용량. 내려받기 명령만 문서에 기재 |
| 로그 디렉토리 전체 | 용량. 대표 로그 몇 개만 |
| 영상 원본 mp4 전체 | 용량 |
| 씬 정의 파일 | 이 저장소의 산출물이 아니다 |
