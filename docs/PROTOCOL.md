# 프로토콜과 좌표계 규약

정책 서버 연동, 좌표 변환, 그리퍼 규약을 정리한다.
모든 항목은 실행 중인 서버 또는 러너 소스에서 직접 확인한 것이다 `[확인]`.

---

## 1. 정책 서버 (ZMQ)

전송은 msgpack + msgpack-numpy이며 기본 포트는 `5555`다.
클라이언트는 `groot_client.GrootClient`를 사용한다.

### 1-1. 관측 스키마

```python
obs = {
    "video": {
        "image":       ndarray (1, 1, 224, 224, 3) uint8,
        "wrist_image": ndarray (1, 1, 224, 224, 3) uint8,
    },
    "state": {
        "x":     ndarray (1, 1, 1) float32,
        "y":     ndarray (1, 1, 1) float32,
        "z":     ndarray (1, 1, 1) float32,
        "roll":  ndarray (1, 1, 1) float32,
        "pitch": ndarray (1, 1, 1) float32,
        "yaw":   ndarray (1, 1, 1) float32,
        "gripper": ndarray (1, 1, 2) float32,
    },
    "language": {
        "annotation.human.action.task_description": [[ "지시문 문자열" ]],
    },
}
```

서버가 거부하는 대표적인 오류와 메시지는 다음과 같다.

| 잘못된 형태 | 서버 응답 |
|---|---|
| `language`가 문자열 | `Observation 'language' must be a dictionary` |
| `language` 키 누락 | `Observation must contain a 'language' key` |
| video가 3차원 | `Video key 'image' must be a numpy array of shape (B, T, H, W, C)` |
| state를 `end_effector_position`으로 전달 | `State key 'x' must be in observation` |
| state가 float64 | `State key 'x' must be a numpy array of type np.float32` |

정리하면 다음 세 가지가 흔한 실수다.

1. **`state`는 축별로 분리되어 있다.** 3차원 벡터 하나가 아니다.
2. **`state`의 dtype은 float32여야 한다.** NumPy 기본값은 float64다.
3. **`language`는 딕셔너리이며 키 이름이 평탄화되어 있다.**

### 1-2. 행동 응답

```python
act = {
    "x": ndarray (1, 16, 1) float32,
    "y": ..., "z": ..., "roll": ..., "pitch": ..., "yaw": ...,
    "gripper": ndarray (1, 16, 1) float32,
}
```

**청크 길이는 16이다.** 현재 구성은 그리퍼가 열린 구간에서 16개 중 2개만,
닫힌 구간에서 8개만 실행하고 나머지를 버린다. 추론 비용을 해석할 때
이 사실을 함께 고려해야 한다.

### 1-3. depth는 정책에 입력되지 않는다

체크포인트의 관측 스키마는 RGB 전용이다. depth는 물체 좌표 추정에만 사용된다.

### 1-4. 물체 좌표도 정책에 입력되지 않는다

검출로 얻은 3D 좌표는 정책의 입력 텐서에 들어가지 않는다. 대신 오프셋을 만들어
두 곳에 사용한다.

```
OFF = RbT · (물체_추정_월드좌표 − 로봇베이스) − LIB_OBJ − bias
```

1. 에피소드 시작 시 팔을 세울 위치
2. 정책에 보고하는 End-Effector 위치의 기준 프레임

즉 좌표를 입력하는 것이 아니라 **좌표로 상태 프레임을 정렬한다.**
그 프레임에서 보면 물체는 항상 학습 시 위치에 있다.

---

## 2. 검출 서버 (ZMQ)

기본 포트 `5556`. 자연어 프롬프트를 받아 박스와 점수를 돌려준다.
모델은 `google/owlv2-base-patch16-ensemble`이다.

에피소드당 1회 호출되며, §4의 정책 호출 지연 표에는 포함되지 않는다.

---

## 3. 좌표계

```
월드          시뮬레이터 기준
로봇 베이스    bp = robot.get_world_pose(),  Rb ≈ diag(−1, −1, 1),  RbT = Rb 전치
정책 프레임    lp = RbT · (ee − bp) − OFF
카메라        USD prim 월드 변환, 광학축 규약 USD (+X right, +Y up, −Z forward)
              fx = fy = 469.27  (1280 × 720)
```

조준 보정과 카메라 오프셋의 부호 규약은 다음과 같다.

```
+x   로봇에서 멀어지는 방향
−y   로봇 기준 오른쪽
+z   위
```

### 3-1. 조준 보정은 베이스 프레임, 검출 오차는 월드 프레임

조준 보정은 다음과 같이 적용된다.

```python
est = est + np.array([-aim[0], -aim[1], aim[2]])   # 베이스 → 월드
```

**조준 보정의 x를 키우면 월드 dx는 작아진다.** 두 프레임을 혼동하면 부호가
뒤집힌다. 보정값은 계산으로 정하지 말고 단일 판 측정으로 확인해야 한다.

### 3-2. 상수 편향이 폐루프에서만 흡수된다

상태 프레임에 더해지는 상수 편향은 폐루프 정책이 스스로 흡수한다.
편향을 213 mm 범위로 변화시켜도 파지점이 4 mm만 변한다(기울기 0.004).

개루프 IK는 흡수하지 않는다. IK 목표점을 계산할 때 이 편향을 되더하지 않으면
목표가 약 217 mm 어긋난다. 실측으로 확인한 값은 다음과 같다.

| 목표점 정의 | 참 좌표와의 오차 |
|---|---|
| 편향 포함 | 216.9 mm |
| 편향 제거 | 0.2 mm |

---

## 4. 그리퍼 규약

정책 출력에서 관절 명령까지의 변환은 다음과 같다.

```
ch      = -sign(2 * gripper - 1)
g_run   = -ch
닫기 조건: g_run < -0.3  (디바운스 후)
```

따라서 **`gripper = 0.0`이 닫기, `gripper = 1.0`이 열기다.**
직관과 반대이므로 다른 정책을 붙일 때 주의해야 한다.

닫기는 지정한 스텝 이전에는 무시되고, 파지 후 지정한 스텝 동안 유지된다.

---

## 5. 한 스텝의 전체 경로

```
[1] 관측 구성
    wrist RGB (1280×720) --crop--> 224×224   (두 슬롯 모두 wrist 영상)
    EE 위치 --> lp = RbT(ee − bp) − OFF
    지시문 --> language

[2] 정책 호출  (K회 호출 후 중앙값)

[3] 청크 구성
    ch[:,0:3] = [x, y, z]                병진 델타
    ch[:,3]   = -sign(2·gripper − 1)
    ch[:,3:6] (회전) = 0                 사용하지 않음

[4] substep 실행  (열림 구간 2, 닫힘 구간 8)
    target_p += ch[k,:3] · ASCALE · RbT
    작업공간 클램프 적용
    하강 하한 적용
    IK → 관절 명령

[5] 그리퍼 디바운스 및 유지

[6] 종료 판정
    상승 > 0.08 m → 성공
    물체 전도 → 조기 종료
    최대 스텝 도달
```

**정책이 생성하는 것은 [2]뿐이다.** [3]의 회전 폐기, [4]의 클램프와 하강 하한,
[5]의 디바운스는 모두 하네스가 수행한다.
