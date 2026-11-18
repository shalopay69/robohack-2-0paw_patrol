# ROBOHACK 2.0 — Unitree Go1

Docker-образ участника для Т1 и Т2. Основа — Ubuntu 22.04, ROS 2 Humble и Cyclone DDS. Контейнер запускается от пользователя `participant`.

## Сборка и проверка Т1

Запустите сборку из корня репозитория в Ubuntu 22.04 (WSL).

```bash
docker buildx build --platform linux/amd64 --load -t go1-participant:t2-local .
timeout 10s docker run --rm --pull=never --network none \
  --entrypoint /entrypoint.sh go1-participant:t2-local ping
```

Сборка загружает интерфейсы Unitree и модель MuJoCo из ревизий, закреплённых в `Dockerfile`. `ping` должен завершиться с кодом `0` за 10 секунд и вывести ровно один JSON-объект.

```json
{"protocol_version":"1.0","supported_tasks":["T2"],"self_trained_rl":false,"preferred_speed_mps":null}
```

## Т2. Блок безопасности

`/entrypoint.sh T2` запускает ROS 2-узел. Он принимает `unitree_go/msg/LowCmd` на `/lowcmd_bare` и публикует проверенные команды на `/lowcmd`.

- Кадр с `NaN`, `Inf` или отрицательными коэффициентами регулятора отбрасывается целиком.
- Целевые углы ограничены диапазонами модели. Шаг от предыдущей цели — до 5° на сустав.
- MuJoCo проверяет переход по 655 парам геометрий с шагом до 0,1°. При самоколлизии глубже 1 мм узел сохраняет предыдущую безопасную цель.
- При потере входных сообщений узел за 0,5 с симуляционного времени переходит в удержание. Последние целевые углы сохраняются, а `dq` и `tau` обнуляются. Публикация `/lowcmd` продолжается.

## Запуск и тесты Т2

Из каталога публичного симулятора запустите MuJoCo вместе с контейнером участника.

```bash
python3 run_local.py --task T2 --headless \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image go1-participant:t2-local
```

Оба контейнера используют один `ROS_DOMAIN_ID`. До первой команды узел удерживает начальную позу модели. Остановка — `Ctrl+C`.

Тесты фильтра без сети. Команду запускайте из корня репозитория решения.

```bash
docker run --rm --pull=never --network none --entrypoint python3 \
  -e PYTHONPATH=/opt/go1:/opt/go1/python \
  --mount "type=bind,src=$PWD/tests/test_safety.py,dst=/test_safety.py,readonly" \
  go1-participant:t2-local /test_safety.py
```

Проверка частоты `/lowcmd`, интервалов публикации и удержания с публичным симулятором.

```bash
bash tests/check_ros_stream.sh ~/go1-simulator-public
```

## Сдача

В заявке укажите ссылку на коммит и команду `docker pull` для образа из этого коммита с `@sha256:…`. [Инструкция организаторов](https://github.com/Urika000/go1-simulator-public/blob/main/docs/Инструкция_по_загрузке_решения.md)
