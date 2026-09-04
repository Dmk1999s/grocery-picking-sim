# grocery-picking-sim

**이마트·홈플러스급 대형마트를 디지털 트윈으로 옮기고, 그 안에서 AMR + 로봇팔이 요청받은 상품을 선반에서 집어오게 하는 Isaac Sim 프로젝트.**

`Python → USD → Isaac Sim` · 환경을 손으로 만들지 않고 **치수 상수에서 생성**한다 · 생성 결과는 **되읽어 숫자로 검증**한다

<p align="center">
  <img src="docs/img/isaac_stocked_overview.png" width="720" alt="Isaac Sim 렌더 — 상품이 채워진 매장 전체">
  <br>
  <sub>Isaac Sim RTX 렌더. 파이썬이 생성한 USD 를 그대로 올린 것. 25.6 × 17.6 m, 부통로 8개, 진열대 137대, YCB 스캔 상품 9,864개</sub>
</p>

<p align="center">
  <img src="docs/img/drive_arm_chase.gif" width="480" alt="Carter + Franka 가 주문 경로를 주행하며 상품을 집어 바구니에 넣는다">
  <br>
  <sub>Isaac Sim 물리 시뮬. Carter v1 + Franka 가 seed 7 주문 4번의 경로 76 m 를 따라 4곳에 정차해 상품을 집어 바구니에 넣고 도크로 돌아온다 (20배속). 시도한 파지 3/3 성공, 정차 오차 ≤ 3.7 cm, 충돌 0</sub>
</p>

<p align="center">
  <img src="docs/img/isaac_stocked_aisle.png" width="352" alt="부통로 안 눈높이">
  <img src="docs/img/isaac_stocked_shelf_face.png" width="352" alt="곤돌라 정면 — 팔 카메라 시야">
  <br>
  <sub>왼쪽: 부통로 0 안, 사람 눈높이. 왼편이 벽면 진열대(2.4 m), 오른편이 곤돌라(1.8 m). 오른쪽: 곤돌라 정면, 로봇 팔 카메라가 볼 시야. 상품은 실물 스캔(YCB)</sub>
</p>

---

## 목표

- **디지털 트윈** — 대형마트 식품 매장을 Isaac Sim 안에 재현한다. 규모는 매장 전체, 실측은 부통로 1~2개 (곤돌라 규격과 통로 폭은 매장 안에서 반복되므로 한두 통로 값이 전체에 적용된다)
- **상품 피킹** — AMR이 통로를 주행하고, 팔이 선반에서 지정 상품을 찾아 집는다
- **시나리오 생성** — 같은 생성기로 seed 기반 변형(가림·배치·조명)을 만들어 인지·파지 성능을 평가한다
- **검증 가능한 환경** — "잘 만들어진 것 같다"가 아니라, 생성물을 되읽어 치수·통행 가능성을 자동으로 확인한다

## 핵심 아이디어 — 환경은 제작물이 아니라 생성기다

```
scene/constants.py ──┬──→ 디지털 트윈    실측값 고정 배치 (실제 마트 재현)
  (치수 단일 진실)    └──→ 시나리오 생성  seed 기반 랜덤 (가림·기울기·조명 변화)
```

- 치수 상수 하나를 바꾸면 진열대 형상 · 매장 배치 · 상품 슬롯 · 검증 기준이 **전부 따라온다**
- 디지털 트윈은 생성기의 **파라미터가 고정된 한 경우**일 뿐이라, 트윈과 시나리오를 둘 다 얻는 데 추가 비용이 거의 없다
- 실측 전에는 시판 곤돌라 규격과 매장 설계 관행에서 가져온 **잠정값**으로 돌고, 실측 후 값만 교체한다
- 모든 상수에 출처를 표시한다: `[실측]` `[잠정]` `[표준]` `[설계]`

## 설계 원칙

**CAD · Blender를 쓰지 않는다.**
마트 구조물은 직육면체의 조합이다. 파이썬 상수가 CAD 스케치보다 덜 정확할 이유가 없고, `CAD → STEP → 메시 → Blender → USD` 변환 사슬에서 UV·머티리얼이 깨져 어차피 다시 손대게 된다. 파라메트릭 CAD의 가치는 구속조건·공차·어셈블리인데 마트에는 그런 게 없다. 대신 파이썬이 USD를 직접 써내면 **실측값 반영이 즉시 되고, 랜덤화가 같은 코드로 된다.**

**상품은 모델링하지 않는다.**
실물을 스캔한 공개 데이터셋을 슬롯에 배치한다. 실제 매장 SKU를 모델링하는 것은 끝이 없고 인지 성능에도 도움이 안 된다.
- [YCB Object and Model Set](https://registry.opendata.aws/ycb-benchmarks/) — 77종, 물리 속성 포함, 매니퓰레이션 표준 벤치마크
- [Google Scanned Objects](https://research.google/blog/scanned-objects-by-google-research-a-dataset-of-3d-scanned-common-household-items/) — 1,030종, CC-BY 4.0

**생성기는 Isaac Sim 없이 돈다.**
`usd-core`만으로 노트북에서 USD를 만들고 Isaac Sim은 결과를 읽기만 한다. 무거운 시뮬레이터를 띄우지 않고 형상을 반복 수정할 수 있다.

**만든 것은 반드시 되읽어 검증한다.**
생성기와 검증기는 같은 상수를 보지만, 검증기는 생성 코드가 아니라 **USD 파일의 실제 형상(AABB)** 을 읽는다. 회전 방향이나 오프셋 실수는 눈으로 보면 놓치지만 숫자 대조는 놓치지 않는다.

**설계는 양팔, 구현은 한 팔.**
마운트 자리는 처음부터 두 개 잡아두되 한 팔로 먼저 완주한다. 두 번째 팔은 "가림 시나리오의 몇 %가 단일 팔로 안 풀리는지" 측정한 뒤에 붙인다.

## 실제 마트 구조를 어떻게 반영했나

디지털 트윈이 목표라서 "상자 몇 개"가 아니라 **실제 곤돌라 진열대와 매장 구성의 구조**를 그대로 따른다. 치수는 실측 전이라 잠정값이지만, 구조는 실측 후에도 바뀌지 않는다.

### 진열대 (곤돌라 한 면)

<p align="center">
  <img src="docs/img/isaac_shelf.png" width="400" alt="곤돌라 진열대 한 면 — Isaac 렌더">
  <img src="docs/img/shelf_3d.gif" width="300" alt="형상 확인용 3D">
</p>

```
       ┌─백판─┐
  지주 │      │ 지주       측면이 뚫려 있다 — 팔이 옆에서 진입할 수 있고
       ├──────┤  상단 선반    카메라가 옆 진열대 너머를 본다
       ├──────┤  (얕다)
       ├──────┤
    ┌──┴──────┤  바닥 데크 (깊다) — 위 단 상품이 통로에서 10 cm 안쪽에 있다
    │  걸레받이 │                    → 팔 도달 거리 계산에 직접 들어간다
  ──┴─────────┴──
```

- **뒤쪽 지주 2개 + 백판** — 통판 측판이 아니다. 측면 개방이 파지 계획과 시야에 영향을 준다
- **바닥 데크는 깊고(500), 상단 선반은 얕다(400)** — 데크 아래 걸레받이는 앞면이 들어가 있다
- **가격표 레일** — 각 단 앞에 4 cm 턱. 상품을 앞으로 끌어낼 때 걸리는 지점이라 파지 전략에 관계된다
- **선반 높이는 지주 홀 피치(25 mm) 배수** — 실제 브래킷이 걸리는 위치로 스냅된다
- **단마다 여유 높이가 다르다** — 최상단은 위에 판이 없다. 하나의 값으로 퉁치면 안 들어가는 상품을 배치하게 된다
- 슬롯마다 "들어갈 수 있는 상품 최대 치수"를 같이 들고 있어 배치 단계에서 안 맞는 에셋을 거른다

### 상품 (YCB 실물 스캔)

- **모델링하지 않고 스캔 데이터셋을 쓴다.** YCB 구글 스캔 메시 85개를 받아 USD 로 바꾸고, 그중 마트에서 팔 만한 32종(캔·박스·병·과일·생활용품·주방·문구·완구·스포츠)만 쓴다
- 에셋마다 **치수는 메시에서, 질량은 YCB 논문 표에서** 가져와 강체·볼록껍질 콜라이더와 함께 붙인다. 선반 위에 놓으면 그대로 물리가 돈다
- **진열 관행대로 채운다.** 통로마다 품목군(플래노그램), 같은 상품을 옆으로 이어 붙이고(페이싱), 앞뒤로 여러 개 세운다
- **트윈과 시나리오가 같은 코드다.** seed 없이 돌리면 결정적 배치, seed 를 주면 상품 순서·빈 자리가 흔들린다. 그 위에 `scenario.py` 가 넘어짐·오배치·조명을 얹는다 (아래)
- 인스턴싱으로 같은 상품 수천 개가 메시 하나를 공유한다. 9,864개를 올려도 렌더가 2~3분

### 매장 (대형마트 식품 매장 한 층의 일부)

<p align="center">
  <img src="docs/img/store_plan.png" width="520" alt="매장 평면도">
  <img src="docs/img/store_3d.gif" width="300" alt="형상 확인용 3D">
  <br>
  <sub>USD 형상에서 뽑은 평면도. 파랑=벽면 진열대, 주황=양면 곤돌라, 빨강=엔드캡, 검정=기둥, 점선=조명, 초록=AMR (0.6 × 0.7 m). 기둥이 떨어진 자리는 진열대를 비운다</sub>
</p>

| | 잠정값 | 근거 |
|---|---|---|
| 부통로 | 8개 × 2.2 m | 대형마트 부통로 2.0~2.4 m, 카트 두 대 교행 |
| 주통로 | 앞·뒤 3.5 m | 대형마트 주통로 3.0~4.0 m |
| 곤돌라 열 | 8대 × 1.2 m = 9.6 m | 열 길이 10 m 내외 |
| 천장 | 5.0 m | 노출 천장 4.5~6 m |
| 기둥 | 0.6 m 각, 9.6 × 8.4 m 그리드 | 곤돌라 열 간격의 배수로 두어 열 위에 오게 (설계 관행) |
| 바닥 | 25.6 × 17.6 m = 451 m² | 부통로 8개가 만드는 크기 |

- **주통로 / 부통로** — 주통로는 넓고(AMR 회전 구간), 부통로는 진열대 사이
- **양면 곤돌라** — 등을 맞댄 두 면. 부통로 사이에 선다
- **엔드캡** — 곤돌라 열 양 끝, 주통로를 보는 단면 진열대 (행사 상품 자리)
- **벽면 진열대** — 벽에 붙고 곤돌라보다 높다 (2.4 m)
- **기둥** — 건물 구조 그리드. 열 위에 오면 그 자리 진열대를 비우고, 통로에 걸치면 통행 검사에 잡힌다
- **천장 + 통로별 라인 조명** — 시나리오 생성기가 세기·색온도를 흔들 축
- **60 cm 타일 바닥** — 실측할 때 타일이 자(尺)가 된다 (`docs/SURVEY.md`). 화면에서도 셀 수 있게 텍스처로 깐다
- 매장 좌표 원점은 **바닥 모서리** — 실측 때 벽 모서리에서 재는 것과 같다
- 프림 이름이 곧 작업 지시 단위: `/World/Shelves/Aisle_00/L/Unit_02` = 부통로 0 왼쪽 면 세 번째 진열대

### 시나리오 (seed 하나 = 어느 날의 매장 + 피킹 주문)

<p align="center">
  <img src="docs/img/scenario_plan.png" width="720" alt="시나리오 평면도 — 주문 경로와 정차·파지 위치">
  <br>
  <sub>seed 7. 주문 5건의 경로(색), 정차 위치(번호 = 방문 순서), 정차점에서 상품 중심으로 팔이 들어가는 선. × 넘어진 상품, ▲ 오배치, 회색 실선 = 꺼진 등. 주통로 주행선은 기둥을 피해 잡는다</sub>
</p>

트윈은 플래노그램대로 꽉 찬 이상적인 매장이다. 학습·평가에는 "어느 날의 매장"이 필요하고, 그건 재현 가능해야 한다. `scene/scenario.py` 는 seed 하나로 아래 전부를 결정하고 USD + JSON 으로 낸다.

| | 무엇을 | 어떻게 |
|---|---|---|
| 매장 상태 | 빈 자리 15 %, yaw 흔들림(±15°), 앞으로 넘어진 상품, 다른 통로 상품이 잘못 놓인 자리, 통로별 조명 세기·꺼진 등 | `stock.plan(seed)` 위에 슬롯(앞뒤 열) 단위로. 회전한 상품이 슬롯에 안 들어가거나 뒤 상품과 겹치면 그 흔들림만 포기 → 물리 검증(verify_stock 7항목)을 그대로 통과한다 |
| 피킹 주문 | 주문 N건 × 품목 M개 | 각 품목은 매장 안 **실제 프림 하나**를 가리킨다. 정답은 USD 의 `stock:*` 속성 — 오배치 상품도 라벨은 실제 상품이라 "플래노그램은 A, 실제는 B" 를 검출기가 맞혀야 한다 |
| 로봇 작업 | 품목마다 정차 자세 (x, y, yaw) · 팔 마운트 · 파지 대상(월드 좌표, 진입 방향) · 도달 거리 | 진열대 앞면에서 `pick_standoff` 만큼 떨어져 왼쪽(팔)이 진열대를 보게 선다. 상품 좌표는 constants 가 아니라 **방금 쓴 USD 를 되읽어** 잡는다 |
| 경로 | 도크 → 정차들 → 도크 경유점, 길이 | 부통로 중심선 + 주통로 주행선 그래프. 방문 순서는 탐욕 최근접 (최적 아님, 로봇 쪽에서 바꿔도 된다) |

**팔 도달이 설계를 돌려 말한다.** 마운트 0.35 m · 도달 0.85 m (UR5e 급) 이면 맨 앞 상품 3,100개 중 도달 가능한 것이 1,774개 (57 %) 다. 3단(1.16 m) 이상은 전부 못 닿는다. 주문은 도달 가능한 상품에서만 뽑되 이 수치를 통계로 남긴다 — 팔을 높이 올리거나(리프트) 더 긴 팔을 쓰라는 근거가 된다.

### 주행 (Isaac Sim 물리 시뮬레이션)

<p align="center">
  <img src="docs/img/drive_plan.png" width="440" alt="계획 경로 vs 실제 궤적">
  <img src="docs/img/drive_pick_chase.png" width="300" alt="정차 순간, 체이스 카메라">
  <br>
  <sub>왼쪽: 계획(초록)과 물리 시뮬 궤적(검정 점선). + 는 실제 정차점. 첫 정차 앞의 작은 고리는 제자리 회전 뒤 3 cm 이상 밀렸을 때 한 번 더 다가간 흔적. 오른쪽: 정차 순간. 왼쪽(팔 마운트)이 진열대를 본다</sub>
</p>

`tools/drive_isaac.py` 가 시나리오 JSON 의 경유점을 Isaac Sim 안에서 **Carter v1** (Isaac 기본 에셋, 차동구동 + 뒤 캐스터) 으로 실제로 주행한다. 계획은 계획이고, 여기서 나오는 건 물리 결과다.

| | seed 7 · ORD_02 |
|---|---|
| 계획 / 주행 거리 | 62.7 m / 64.7 m |
| 시뮬 시간 / 벽시계 | 183 s / 161 s (렌더 포함) |
| 정차 4곳 위치·yaw 오차 | 최대 3.7 cm · 2.0° |
| 본체 ↔ 장애물 최소 간격 | 6.7 cm (정차 앞 제자리 회전 때 모서리가 진열대 쪽으로 쓸린다) |
| 충돌 프레임 / 도크 복귀 오차 | 0 / 2.7 cm |

- **제어는 단순 유니사이클 추종기**: 경유점을 향해 제자리 회전 → 직진(도착 근처 감속) → 정차점에서는 지정 yaw 로 회전. 위치는 시뮬 참값 — 로컬라이제이션은 아직 없다
- **바퀴 부호를 스스로 잡는다.** 시작할 때 0.75 s 굴려 보고 heading 방향으로 갔는지 본다. 에셋마다 조인트 축 방향이 다르기 때문
- **간격은 검증기와 같은 소스로 잰다.** 본체 둘레 표본점과 USD 에서 읽은 진열대·기둥·벽 AABB 사이 거리를 10 Hz 로 기록한다. 정차 앞 제자리 회전 때 6.7 cm 까지 좁아지는 것은 사각형이 도는 기하학적 한계다 (standoff 0.20 − 모서리 반경 차 0.145)
- `tools/verify_drive.py` 가 결과 JSON 을 10항목으로 본다: 완주, 정차 오차, 충돌 0, 거리 비율, 도크 복귀, 궤적이 바닥 안
- 팔은 아직 없다. 정차 자세·파지 대상 좌표는 JSON 에 있으니 다음은 그 자리에서 팔을 뻗는 것

### 팔 (Carter 위 Franka — 정차 자세에서 집어 바구니에)

<p align="center">
  <img src="docs/img/arm_grasp_close.png" width="352" alt="스팸 캔 파지 순간">
  <img src="docs/img/arm_over_bin.png" width="352" alt="바구니 위로 옮기는 중">
  <br>
  <sub>왼쪽: 맨 앞 스팸 캔을 통로 방향으로 집는 순간 (뒤 상품이 1 cm 뒤에 붙어 있어 깊이 방향으로는 손가락이 못 들어간다). 오른쪽: 세정제를 상판 뒤 바구니로 옮기는 중. 손은 수평을 유지한다 — 아래로 돌리면 둥근 캔이 빠졌다</sub>
</p>

<p align="center">
  <img src="docs/img/reach_study.png" width="520" alt="팔 어깨 높이 × 도달 반경 → 닿는 상품 비율">
  <br>
  <sub>팔 사양을 감으로 정하지 않기 위한 스터디. Franka(0.855 m)를 Carter 상판에 얹으면 어깨 1.01 m — 바닥 데크(0단)는 못 닿고 나머지 단은 닿는다 (맨 앞 상품의 77 %)</sub>
</p>

<p align="center">
  <img src="docs/img/drive_arm_plan.png" width="520" alt="팔 포함 주행 — 계획 vs 실제 궤적">
  <br>
  <sub>팔 포함 주행의 계획(주황)과 실제 궤적(검정 점선). + 는 실제 정차점</sub>
</p>

`tools/arm_isaac.py` 가 Franka Panda 를 Carter 상판에 얹고, 정차마다 상품을 집어 상판 뒤 바구니에 넣는다. 팔은 별도 관절체를 매 스텝 Carter 자세로 옮겨 태운 것이다 — 정차 중엔 베이스가 서 있으므로 고정 베이스 팔과 같고, 주행 중 상품은 바구니에 있다.

| | seed 7 · ORD_04 |
|---|---|
| 순간이동 모드, 주문 5건 정차 20곳 | 시도 18 중 **16 성공 (89 %)**. 미시도 2는 물리 시작 때 넘어진 세정제 — "지금 자세로는 못 집음" 판정. 실패 2는 둘 다 머스터드 병(둥글고 0.43 kg)이 빼내는 중 빠진 것. 단별로는 1~4단 전부 성공 사례 있음 |
| 주행 포함 (76.5 m, 정차 4곳) | 시도한 파지 3/3 성공 → 바구니. 정차 오차 ≤ 3.7 cm / 2.0°, 충돌 0, 시뮬 249 s / 벽시계 391 s (렌더 포함). 검증 15항목 통과 |
| 시퀀스 | tuck → 프리그래스프 → 접근 → 닫기(70 N) → 4 cm 들기 → 빼기 → 위로 → 바구니 위 → 두 단계 놓기 → tuck, 약 15 s |
| 손끝 도달 오차 | ≤ 1.5 mm (관절 드라이브 강성 ×4 전에는 1.5 cm) |

- **어디를 집을지**: 맨 앞 상품, 손가락은 통로 방향으로 닫힌다. 그래서 통로 방향 폭 ≤ 7 cm, 높이 ≥ 7 cm(손 몸통이 선반에 닿지 않게)인 상품만 주문에 들어온다 — 맨 앞 상품의 43 %. 나머지는 흡착이나 위에서 집기가 필요하다는 뜻이고, 그 숫자가 `scenario_NNN.json` 통계에 있다
- **파지점은 계획이 아니라 지금 상품 자세로** 다시 잡는다 (인식 대용). 물리가 시작되면 넘어지는 상품이 있다. 폭·높이가 규칙 밖이면 `not_graspable_now` 로 기록한다
- **파지가 되기까지 잡은 것**: 에셋 오른쪽 손가락에 드라이브가 없던 것(7 N → 70 N 양쪽), 관절 보간이 선반을 치던 것(직교 좌표 직선 + slerp), 카레 중 IK 반전(위로 → 바구니 위 → 내리기), 손끝 처짐 1.5 cm(강성 ×4), **손가락 질량 14 g vs 상품 0.4 kg**(50 g 으로 → 미끄러짐 2.3 cm → 0.2 cm), 손을 아래로 돌리면 캔이 빠지는 것(수평 유지), 놓을 때 튕김(두 단계). 전부 `docs/LOG.md` (7)
- 순간이동 모드(`--teleport`)는 주행을 건너뛰고 정차 자세로 바로 옮겨 1.5 분에 4회 파지를 돌린다. 파지 물리를 잡는 데 15번쯤 썼다

## 검증

생성할 때마다 자동으로 175항목을 대조한다. 실패하면 종료 코드 1이라 CI에 바로 걸 수 있다.

| 검증기 | 항목 수 | 보는 것 |
|---|---|---|
| `tools/verify_shelf.py` | 58 | 외형 치수, 지주 위치·측면 개방, 단별 높이·두께·앞단 위치, 레일, 홀 피치 스냅, 콜라이더, 슬롯 내부 여부, 대표 상품(캔·크래커·병) 적합성 |
| `tools/verify_store.py` | 82 | 바닥·천장·벽·기둥·조명, 진열대 대수·높이·바닥 접촉·벽 내부, 상호 겹침(137대 쌍 검사), 기둥 간섭, 앞면이 통로 경계에 있는지, **AMR 직진 여유폭 · 제자리 회전 · 통로 입구 회전 가능성** (통로 10개 × 입구 16곳) |
| `tools/verify_stock.py` | 7 | 상품 9,864개 전부: 참조가 풀려 메시가 있는지, 강체·콜라이더·질량, 밑면이 선반에 닿는지(±2 mm), 자기 슬롯 안(폭·깊이·높이)인지, 같은 진열대 안 겹침, 메타데이터 중복 |
| `tools/verify_scenario.py` | 18 | 시나리오 JSON ↔ USD: 주문 품목의 프림·라벨 일치, 맨 앞 상품인지, 파지 좌표 = USD AABB 중심, **팔 도달**, 정차 본체+안전여유가 장애물과 안 겹침, 본체 ↔ 앞면 간격 = standoff, 경로 띠가 장애물과 안 겹침, 흔들림 라벨(오배치는 다른 품목군, 넘어짐은 rotateY −90)·조명 값·개수 통계, **같은 seed 재생성 시 JSON 동일** |
| `tools/verify_drive.py` | 10 | Isaac 주행 결과: 완주, 정차 수 = 품목 수, 위치 오차 ≤ 5 cm, yaw ≤ 3°, 충돌 0, 최소 간격 ≥ 0, 주행/계획 거리 비율, 도크 복귀, 궤적이 바닥 안 |

통행 검사는 상수가 아니라 USD 안의 실제 형상으로 한다. 기둥·엔드캡·가격표 레일이 통로로 튀어나온 만큼을 전부 반영한 뒤, AMR 본체 + 안전 여유가 들어가는지 본다.

## 현재 상태

| | | |
|---|---|---|
| `scene/constants.py` | ✅ | 치수 단일 진실 공급원. 출처 태그 부착, 대부분 `[잠정]` |
| `scene/shelf.py` | ✅ | 곤돌라 진열대 생성기 (지주·백판·데크·걸레받이·선반·레일, 슬롯 좌표) |
| `scene/store.py` | ✅ | 매장 생성기 (대형마트 규모, 주·부통로, 벽면·양면·엔드캡, 기둥 그리드, 조명, 타일 바닥) |
| `tools/verify_shelf.py` | ✅ | 진열대 대조 검증 58항목 |
| `tools/verify_store.py` | ✅ | 매장 대조 검증 + AMR 통행 82항목 |
| `tools/plan_store.py` | ✅ | USD → 평면도 PNG |
| `tools/render_3d.py` | ✅ | USD → 3D PNG / 회전 GIF (matplotlib, 형상 확인용) |
| `tools/render_isaac.py` | ✅ | USD → Isaac Sim RTX 렌더 (헤드리스) |
| `tools/ycb_catalog.py` | ✅ | YCB 스캔 메시 → USD 에셋(텍스처·강체·질량) + 치수 카탈로그 84종 |
| `scene/stock.py` | ✅ | 상품 배치: 플래노그램·페이싱·앞뒤 세우기, seed 시나리오, 인스턴싱 |
| `tools/verify_stock.py` | ✅ | 상품 배치 검증 |
| 실측 | ⬜ | `docs/SURVEY.md` 절차대로 통로 1~2개 |
| `scene/scenario.py` | ✅ | seed → 매장 상태(빈 자리·yaw·넘어짐·오배치·조명) + 피킹 주문 + 정차·파지·경로 JSON |
| `tools/verify_scenario.py` | ✅ | 시나리오 검증 18항목 (도달·정차·경로·재현성) |
| `tools/plan_scenario.py` | ✅ | 시나리오 평면도 (경로·정차·흔들림 오버레이) |
| `tools/drive_isaac.py` | ✅ | Isaac Sim 에서 Carter v1 이 주문 경로를 물리 주행 (체이스·1인칭 녹화, GIF) |
| `tools/verify_drive.py` | ✅ | 주행 결과 검증 10항목 |
| `tools/arm_isaac.py` | ✅ | Carter 위 Franka: 정차 자세에서 집어 바구니에 (Lula IK, 순간이동 실험 모드) |
| `tools/reach_study.py` | ✅ | 팔 어깨 높이 × 도달 반경 스터디 |
| 인식 | ⬜ | 지금은 시뮬 참값(현재 AABB). 카메라 → 검출 → 파지점 |

## 쓰는 법

```bash
python3 -m venv .venv && .venv/bin/pip install usd-core matplotlib   # matplotlib 은 평면도용

.venv/bin/python -m scene.shelf --out out/shelf.usda          # 진열대 생성
.venv/bin/python -m tools.verify_shelf out/shelf.usda         # 58항목 검증

.venv/bin/python -m scene.store --out out/store.usda          # 매장 생성 (+ floor_tile.png)
.venv/bin/python -m tools.verify_store out/store.usda         # 82항목 검증
.venv/bin/python -m tools.plan_store out/store.usda           # 평면도 → out/store_plan.png
.venv/bin/python -m tools.render_3d  out/store.usda --gif out/store_3d.gif   # 3D 회전 GIF

# 상품: YCB 스캔 메시 받기 (600 MB) → USD 에셋 + 카탈로그 → 슬롯에 채우기 → 검증
aws s3 sync --no-sign-request --exclude "*" --include "*_google_16k.tgz" s3://ycb-benchmarks/data/google/ assets/ycb/raw/
.venv/bin/python -m tools.ycb_catalog
.venv/bin/python -m scene.stock --store out/store.usda --out out/store_stocked.usda   # 트윈 (결정적)
.venv/bin/python -m scene.stock --seed 7 --fill 0.7 --out out/stock_seed007.usda     # 상품만 흔든 버전
.venv/bin/python -m tools.verify_stock out/store_stocked.usda

# 시나리오: seed → 매장 상태 + 주문 + 정차·경로 (USD + JSON) → 검증 → 평면도
.venv/bin/python -m scene.scenario --seed 7 --orders 5 --lines 4      # out/scenario_007.usda + .json
.venv/bin/python -m tools.verify_stock out/scenario_007.usda            # 흔들린 상태도 물리적으로 말이 되는지
.venv/bin/python -m tools.verify_scenario out/scenario_007.json         # 18항목 (재생성 비교 포함, ~10 s)
.venv/bin/python -m tools.plan_scenario out/scenario_007.json           # → out/scenario_007_plan.png
```

Isaac Sim은 `out/store_stocked.usda`를 스테이지에 얹기만 하면 된다. 실제 렌더는 Isaac Sim 파이썬으로:

```bash
source ~/.isaac_cache_env   # OMNI_KIT_ACCEPT_EULA=YES 등
~/isaac6-venv/bin/python -m tools.render_isaac out/store_stocked.usda --out out/isaac
```

EC2 에서 Isaac Sim GUI 를 띄워 노트북에서 직접 돌려보려면 WebRTC 스트리밍:

```bash
bash tools/view_isaac.sh out/store_stocked.usda     # EC2 에서. TCP 49100 / UDP 47998 이 열린다
```

주행 (Isaac Sim 물리). 주문 하나에 2~3 분:

```bash
source ~/.isaac_cache_env
~/isaac6-venv/bin/python -m tools.drive_isaac out/scenario_007.json --order 2 --record   # → out/drive/drive_007_ORD_02.json + 프레임 + GIF
.venv/bin/python -m tools.verify_drive out/drive/drive_007_ORD_02.json                    # 10항목
.venv/bin/python -m tools.plan_scenario out/scenario_007.json --order 2 --drive out/drive/drive_007_ORD_02.json   # 계획 vs 실제 궤적

# 팔: 정차마다 집어 바구니에. --teleport 는 주행 없이 정차 자세로 바로 (파지 실험, 1.5 분)
~/isaac6-venv/bin/python -m tools.drive_isaac out/scenario_007.json --order 4 --arm --teleport --record --out out/drive_arm
~/isaac6-venv/bin/python -m tools.drive_isaac out/scenario_007.json --order 4 --arm --record --out out/drive_arm_full
.venv/bin/python -m tools.reach_study out/store_stocked.usda --out docs/img/reach_study.png
```

노트북에는 [Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html) 를 설치하고 인스턴스 공인 IP 로 접속한다. 보안그룹에 내 IP 에서 위 포트 인바운드가 필요하다.

## 로드맵

1. **실측** — 마트 한 곳에서 부통로 1~2개. 타일·상품을 자로 쓰는 절차는 `docs/SURVEY.md`
2. **상품 확장** — YCB 는 마트 상품이 32종뿐이라 통로가 단조롭다. Google Scanned Objects 로 넓힌다
3. ~~**`scenario.py`**~~ — 완료. 다음은 가림(앞 상품이 뒤 상품을 가리는 배치)·기울어짐(90° 가 아닌 각) 추가
4. ~~**AMR 주행**~~ ~~**팔 파지**~~ — 완료 (Carter v1 + Franka). 다음: 인식(카메라 → 검출 → 파지점, 지금은 참값), 로컬라이제이션, 0단 도달(리프트), 폭 > 7 cm 상품(흡착), 납작한 상품(위에서 집기)
5. **μ 스윕** — 선반·그리퍼 마찰계수는 실측 불가능한 값이라 하나로 고정하지 않고 스윕 축으로 둔다

## 레포 구조

```
scene/
  constants.py    치수 단일 진실 공급원 — 여기서 시작한다
  shelf.py        진열대 생성기 + 슬롯 좌표
  store.py        매장 생성기 + 배치·통로 계산
  stock.py        상품 배치 (플래노그램 · seed 시나리오)
  scenario.py     시나리오: seed → 매장 상태 + 주문 + 정차·파지·경로 JSON
tools/
  ycb_catalog.py  YCB 메시 → USD 에셋 + 카탈로그
  verify_stock.py 상품 배치 검증
  verify_scenario.py 시나리오 검증 (도달·정차·경로·재현성)
  plan_scenario.py   시나리오 평면도 (경로 오버레이, --drive 로 실제 궤적)
  drive_isaac.py     Isaac Sim 에서 AMR 주행 + 녹화
  verify_drive.py    주행·파지 결과 검증
  arm_isaac.py       Carter 위 Franka 파지 모듈 (drive_isaac --arm)
  reach_study.py     팔 어깨 높이 × 도달 스터디
  frames_to_gif.py   프레임 → README 용 GIF
  verify_shelf.py 진열대 검증
  verify_store.py 매장 검증 + AMR 통행
  plan_store.py   평면도 렌더
  render_3d.py    3D PNG / GIF 렌더 (matplotlib)
  render_isaac.py Isaac Sim RTX 렌더
  view_isaac.sh   Isaac Sim GUI 를 WebRTC 로 스트리밍 (EC2 → 노트북)
assets/ycb/
  catalog.json    상품 치수·질량·분류 (커밋). 메시·USD 는 받아서 만든다 (gitignore)
docs/
  SURVEY.md       실측 안내 — 무엇을, 어떻게, 얼마나만 잴 것인가
  LOG.md          개발 기록 — 무엇을 왜 했는지, 날짜순
  img/            README 그림 (전부 도구로 재생성 가능)
```

## 개발 기록

결정과 이유는 [`docs/LOG.md`](docs/LOG.md) 에 날짜순으로 남긴다. "이 숫자 어디서 났냐", "왜 이렇게 했냐"에 답하는 곳이다.

## 기술 스택

- **USD** (`usd-core`) — 장면 기술. Isaac Sim 네이티브 포맷
- **NVIDIA Isaac Sim** — 물리·렌더·로봇 (EC2 GPU 인스턴스)
- **Python 3.10+** — 생성기·검증기. 의존성은 `usd-core` 하나 (평면도만 `matplotlib`)

## 참고

- [arXiv:2409.15465](https://arxiv.org/abs/2409.15465) — 선반 로컬 프레임(+X 안쪽, yz 평면 = 통로에서 본 실루엣)의 축 규약을 이 논문의 item frame 과 맞췄다
- [YCB Object and Model Set](https://registry.opendata.aws/ycb-benchmarks/), [Google Scanned Objects](https://research.google/blog/scanned-objects-by-google-research-a-dataset-of-3d-scanned-common-household-items/) — 상품 에셋
