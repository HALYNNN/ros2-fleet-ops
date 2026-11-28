# 운영 런북 — ros2-fleet-ops

현장에서 "로봇이 이상하다"는 연락을 받았을 때, **추측 대신 기록으로** 원인을 좁히는 순서입니다.
모든 명령은 저장소 루트(`/opt/ros2-fleet-ops`)에서 실행합니다.

## 0. 30초 상태 확인

```bash
docker compose ps                       # 세 서비스 healthy 여부
curl -s localhost:8000/api/health | jq  # collector heartbeat 신선도
curl -s localhost:8000/api/robots | jq '.[0].health, .[0].topics'
```

`collector_alive: false`이면 수집기 자체가 멈춘 것이므로 1-B로 갑니다.
로봇은 살아 있는데 `health: stale`이면 토픽이 끊긴 것이므로 1-A로 갑니다.

## 1-A. 알람이 올라온 경우

| 알람 | 먼저 확인 | 흔한 원인 | 다음 조치 |
| --- | --- | --- | --- |
| `topic_silent` | `ros2 node list`, `ros2 topic hz /scan` | 센서 노드 다운, USB/이더넷 분리, DDS 단절 | 노드 재시작 → 동일 구간 Replay로 복구 확인 |
| `topic_drop` | `ros2 topic echo /odom --once`, QoS depth | 발행 측 큐 포화, 수신 측 처리 지연 | QoS/버퍼 확인, 수신 프로세스 CPU 확인 |
| `gap_spike` | 같은 시각대 CPU·네트워크 사용률 | 순간 부하, Wi-Fi 재전송 | 반복되면 노드 우선순위·무선 구간 점검 |
| `latency_high` | 발행/수신 타임스탬프 차이 | 브리지·릴레이 구간, 시계 오차(`use_sim_time`) | 구간 분리 측정, 시간 동기 확인 |
| `rate_low` | `ros2 topic hz`, 노드 파라미터 | 발행 주기 설정 변경, CPU 부하 | 파라미터 원복 또는 부하 분산 |
| `estop_command` | `ros2 topic echo /cmd_vel`, 비상정지 입력 | 관제 정지 명령, 자율주행 노드 종료 | 정지 해제 절차, 명령 발행자 확인 |

`estop_command`는 `cmd_vel`이 0이 된 뒤 `estop_debounce_s`(기본 0.5s) 동안 계속 0인데도 `odom` 속도가 기준(0.02 m/s)을 넘을 때만 발생합니다. 정상 제동은 `cmd_vel`=0이 잠깐 나타나므로 걸리지 않습니다.
`--use-sim-time`으로 돌 때 알람·샘플의 `ts`/`recv_ts`는 시뮬레이션 시간(0 근처)이고, 화면·생존 판정에 쓰는 실제 시각은 `wall_ts`입니다.

```bash
# 현재 run의 알람
curl -s "localhost:8000/api/alarms?limit=50" | jq '.[] | {ts,severity,topic,kind,message}'

# 원인 후보·타임라인 리포트
python -m collector.analyzer --latest
python -m collector.analyzer --latest --markdown --out reports/incident.md
```

## 1-B. 수집기가 멈춘 경우

```bash
cat data/collector.heartbeat          # 마지막 heartbeat의 시각·샘플 수
docker compose logs --tail=100 collector
systemctl status ros2-fleet-ops
```

heartbeat가 오래됐으면 컨테이너가 재시작 중이거나 DB 잠금일 수 있습니다. `docker compose restart collector` 후 `/api/health`로 확인합니다.
재시작해도 run은 닫히지 않고(`open: true`) 이어서 쌓입니다. 종료된 run만 분석 대상으로 삼으려면 `finished_at`을 확인하세요.

## 2. 증거 수집

```bash
RUN=$(python -c "import sys;sys.path.insert(0,'.');from collector.storage import Store;s=Store('data/fleet.db','data/jsonl');print(s.latest_run()['run_id'])")
cp "data/jsonl/$RUN.jsonl" "reports/$RUN.jsonl"     # 원본 샘플·알람
python -m collector.analyzer --run-id "$RUN" --markdown --out "reports/$RUN-report.md"
```

* SQLite(`data/fleet.db`)는 조회용, JSONL(`data/jsonl/<run_id>.jsonl`)은 원본 보관용입니다. 이슈 트래커에는 JSONL과 리포트를 함께 첨부합니다.
* 리포트에는 run id, 로봇, 데이터 구간, 토픽별 건수·측정 Hz·최대 간격·p95 지연·결번 수가 들어갑니다. **수치를 손으로 적지 않습니다.**

## 3. Replay로 재현

```bash
curl -s -X POST localhost:8000/api/replay \
  -H 'Content-Type: application/json' \
  -d '{"run_id":"'"$RUN"'","speed":2}' | jq '.state,.total'
```

대시보드에서 run을 고르고 배속을 선택한 뒤 `Replay`를 누르면 궤적이 기록 순서대로 다시 그려집니다.
장애 시각의 궤적·속도와 알람 타임라인을 나란히 놓고 "그 시점에 로봇이 어디서 무엇을 하고 있었는지"를 확인합니다.

## 4. 장애 재현 (수정 전 확인)

```bash
# 시뮬레이터에서 같은 장애를 재현: 임계값·탐지 로직 검증
python scripts/inject_fault.py --faults silence --silence-at 5 --silence-for 4 --duration 15 --analyze

# 실제 ROS 2 스택에서
python scripts/inject_fault.py --mode ros2 --fault estop --duration 3
python scripts/inject_fault.py --mode ros2 --fault silence --target-pid <노드 PID> --duration 5
python scripts/inject_fault.py --mode ros2 --fault delay --delay-ms 300 --src-topic /scan_raw --dst-topic /scan --duration 20
```

지연 relay를 쓰려면 실제 센서 노드를 `/scan_raw`로 remap해야 합니다(`--ros-args -r /scan:=/scan_raw`). 운영 중인 노드에 SIGSTOP을 보내는 것은 로봇을 멈추는 행위이므로, 반드시 정지 상태에서 수행합니다.

## 5. 종료·인계

1. 리포트와 JSONL을 `reports/`에 보관하고 run id를 이슈에 기록
2. 임계값을 바꿨다면 `config/collector.yaml` 변경 사유를 커밋 메시지에 남김
3. 재발 방지 항목(가드레일, 알람 추가, 복구 자동화)을 이슈로 등록
4. 같은 장애를 `inject_fault.py` 시나리오로 추가해 회귀 검증 대상으로 만듦

## 6. 자주 쓰는 명령

```bash
python -m collector.collector --mode sim --duration 30          # 시뮬레이터 수집
python -m collector.collector --mode ros2 --use-sim-time        # Gazebo 구독
python -m collector.analyzer --latest --json                    # 기계 판독용 리포트
python scripts/inject_fault.py --faults latency,drop,silence,estop --duration 30 --analyze
docker compose run --rm fault-injector                          # compose 프로필 도구
curl -s localhost:8000/api/runs | jq '.[0:3]'                   # 주행 기록
```

## 7. 알람 심각도 기준

| 심각도 | 기준 | 대응 |
| --- | --- | --- |
| `critical` | 정지 시간이 기준의 2배 초과, 지연이 예산의 2배 초과 | 즉시 현장 확인·로봇 정지 검토 |
| `warning` | 기준 초과, 단발성 | 당일 내 원인 확인, 추세 관찰 |
| `info` | 복구·상태 변화 | 기록만 |
