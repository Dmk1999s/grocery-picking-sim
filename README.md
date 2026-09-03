# grocery-picking-sim

> 실제 마트를 기반으로 한 Isaac Sim 환경에서, AMR + 로봇팔이 요청받은 상품을
> 선반에서 찾아 집어오게 만든다.

환경을 **제작물이 아니라 생성기로** 다룬다. 치수 상수 하나에서 선반·매장·상품
배치가 전부 파생되고, 같은 생성기가 두 가지를 만든다:

```
constants.py ─┬─→ 디지털 트윈   실측값 고정 배치 (실제 마트 재현)
              └─→ 시나리오 생성  seed 기반 랜덤 (가림·기울기·조명 변화)
```

트윈은 생성기의 **파라미터가 고정된 한 경우**일 뿐이라, 둘 다 얻는 데 추가
비용이 거의 없다.

## 설계 원칙

**CAD 를 쓰지 않는다.** 마트 구조물은 직육면체다. 파이썬 상수가 CAD 스케치보다
덜 정확할 이유가 없고, `FreeCAD → STEP → 메시 → Blender → USD` 변환 사슬에서
UV·머티리얼이 날아가 어차피 다시 만들게 된다. 파라메트릭 CAD 의 가치는
구속조건·공차·어셈블리인데 마트에는 그런 게 없다.

**상품을 모델링하지 않는다.** 실물을 스캔한 데이터셋을 쓴다:
- [YCB Object and Model Set](https://registry.opendata.aws/ycb-benchmarks/) —
  77종, 물리 속성 포함, 매니퓰레이션 표준 벤치마크
- [Google Scanned Objects](https://research.google/blog/scanned-objects-by-google-research-a-dataset-of-3d-scanned-common-household-items/) —
  1,030종, CC-BY 4.0

**생성기는 Isaac 없이 돈다.** `usd-core` 만으로 노트북(macOS)에서 USD 를 만들고
Isaac Sim 은 결과를 읽기만 한다. 무거운 시뮬레이터를 띄우지 않고 형상을 반복
수정할 수 있다.

**설계는 양팔, 구현은 한 팔.** 마운트 자리는 처음부터 두 개 잡아두되 실제로는
한 팔로 먼저 완주한다. 두 번째 팔은 "가림 시나리오의 몇 %가 단일 팔로 안
풀리는지" 측정한 뒤에 붙인다.

## 현재 상태

| | |
|---|---|
| `scene/constants.py` | ✅ 치수 단일 진실 공급원 (대부분 `[잠정]` — 실측 대기) |
| `scene/shelf.py` | ✅ 진열대 생성기 (순수 USD, 슬롯 좌표 포함) |
| `tools/verify_shelf.py` | ✅ 생성 결과 되읽어 대조 검증 (28항목) |
| `scene/store.py` | ⬜ 매장 레이아웃 (통로·벽·기둥·진열대 배치) |
| `scene/stock.py` | ⬜ 상품 배치 (YCB/GSO 를 슬롯에 채움) |
| `scene/scenario.py` | ⬜ seed 기반 시나리오 생성 |
| 로봇 | ⬜ AMR + 단일 팔 |

## 쓰는 법

```bash
python3 -m venv .venv && .venv/bin/pip install usd-core

.venv/bin/python -m scene.shelf --out out/shelf.usda   # 생성
.venv/bin/python -m tools.verify_shelf out/shelf.usda  # 검증
```

### 지금 검증이 FAIL 하는 항목

```
[FAIL] 크래커박스(21cm) 가 들어가는 단   4/5단  ← 단 [4] 불가
[FAIL] 500ml병(23cm) 가 들어가는 단     4/5단  ← 단 [4] 불가
```

**코드 결함이 아니다.** 잠정 치수(`top_z=1.60`, `height=1.80`)에서 최상단 여유가
20 cm 뿐이라 나오는 결과이고, 실측하면 해소된다. `docs/SURVEY.md` 참고.

## 다음 단계

1. 실제 마트 한 곳에서 통로 1~2개 실측 → `constants.py` 갱신 (`docs/SURVEY.md`)
2. `store.py` — 진열대를 열로 배치하고 통로 폭 확보, AMR 통과 가능성 검증
3. YCB 데이터셋 받아서 슬롯 치수에 맞는 물체 선별
4. `stock.py` + `scenario.py` — seed 로 재현 가능한 배치
5. Isaac Sim 에 올려 AMR 주행 → 상품 검출 → 파지
