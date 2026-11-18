# ROBOHACK 2.0 — Unitree Go1

## Сборка

```bash
docker buildx build --platform linux/amd64 --load -t go1-participant:t2-local .
```

## Т1

```bash
timeout 10s docker run --rm --pull=never --network none \
  --entrypoint /entrypoint.sh go1-participant:t2-local ping
```

## Т2

```bash
python3 ~/go1-simulator-public/run_local.py --task T2 --headless \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image go1-participant:t2-local
```

## Тесты

```bash
docker run --rm --pull=never --network none --entrypoint python3 \
  -e PYTHONPATH=/opt/go1:/opt/go1/python \
  --mount "type=bind,src=$PWD/tests/test_safety.py,dst=/test_safety.py,readonly" \
  go1-participant:t2-local /test_safety.py

bash tests/check_ros_stream.sh ~/go1-simulator-public
```
