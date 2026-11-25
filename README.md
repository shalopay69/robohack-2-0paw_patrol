# ROBOHACK 2.0 — Unitree Go1

## Сборка

```bash
docker buildx build --platform linux/amd64 --load -t go1-participant:t4-local .
```

Если образ `go1-participant:release-37ad667` уже есть локально, можно использовать
готовую среду Т2.

```bash
docker buildx build --platform linux/amd64 --load \
  --build-arg RUNTIME_IMAGE=go1-participant:release-37ad667 \
  -t go1-participant:t4-local .
```

## Т1

```bash
timeout 10s docker run --rm --pull=never --network none \
  --entrypoint /entrypoint.sh go1-participant:t4-local ping
```

## Т2

```bash
python3 ~/go1-simulator-public/run_local.py --task T2 --headless \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image go1-participant:t4-local
```

## Т3

Политика Go1 получает скорость через `/cmd_vel` и состояние суставов и IMU через
`/lowstate`. Команды 12 суставов идут в `/lowcmd_bare`. Safety Block из Т2
проверяет их и публикует `/lowcmd`.

```bash
bash tests/check_t3_walk.sh ~/go1-simulator-public go1-participant:t4-local 0.4
```

Команда запускает два контейнера и подаёт 0,4 м/с. Через 1,5 с симуляционного
времени начинается отсчёт дистанции. Результат содержит пройденное расстояние,
время и признак падения. Описание дообучения и журналы лежат
в [training](training/README.md).

## Т4

Узел оценивает горизонтальное положение корпуса по сканам `/scan`. Ориентация
из `/imu/data` нужна для компенсации крена и тангажа и для оценки поворотов.
Простой локальный набор опорных сканов позволяет сопоставлять поверхности
на нескольких соседних позициях. Поза публикуется в `/odom` и TF
`odom → trunk` с частотой 40 Гц симуляционного времени.

```bash
bash tests/check_t4_odometry.sh ~/go1-simulator-public go1-participant:t4-local
```

Проверка запускает Gazebo Highlevel с lockstep. Тестовый процесс подаёт
команды на квадратную траекторию, сравнивает одометрию с эталоном публичного
симулятора и измеряет дрейф за 10 секунд после остановки.

## Тесты

```bash
docker run --rm --pull=never --network none --entrypoint python3 \
  -e PYTHONPATH=/opt/go1:/opt/go1/python \
  --mount "type=bind,src=$PWD/tests/test_safety.py,dst=/test_safety.py,readonly" \
  go1-participant:t4-local /test_safety.py

bash tests/check_ros_stream.sh ~/go1-simulator-public go1-participant:t4-local
```
