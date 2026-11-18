# ROBOHACK 2.0 · Т1

Локальный образ `Participant` на Ubuntu 22.04 с ROS 2 Humble и Cyclone DDS. На этом этапе реализован `ping`. Список `supported_tasks` пуст до подключения рабочих режимов Т2–Т5.

## 1. Сборка

Из каталога с `Dockerfile` в Ubuntu 22.04 под WSL выполните команду:

```bash
docker buildx build --platform linux/amd64 --load -t go1-participant:t1-local .
```

`--platform` задаёт архитектуру стенда, а `--load` загружает результат в локальный Docker Engine. Интернет нужен для получения базового образа и пакетов при сборке.

## 2. Проверка `ping`

```bash
timeout 10s docker run --rm --pull=never --network none \
  --entrypoint /entrypoint.sh go1-participant:t1-local ping
```

Docker запускает контейнер с пользователем `participant`. У команды должен быть код выхода `0` и единственный JSON-объект в `stdout`.

```json
{"protocol_version":"1.0","supported_tasks":[],"self_trained_rl":false,"preferred_speed_mps":null}
```

## 3. Проверка окружения

```bash
docker image inspect go1-participant:t1-local --format '{{.Os}}/{{.Architecture}}'
docker run --rm --pull=never --network none --entrypoint /bin/sh \
  go1-participant:t1-local -c 'id -u && . /opt/ros/humble/setup.sh && ros2 pkg prefix rmw_cyclonedds_cpp'
```

Ожидаемый результат — `linux/amd64`, UID `10001` и путь `/opt/ros/humble` для пакета Cyclone DDS.

## 4. Подготовка к публикации

Для каждого готового задания добавьте его режим запуска в `/entrypoint.sh`, затем внесите идентификатор в `supported_tasks` и проверьте режим с публичным симулятором. Для заявки нужны публичная ссылка на коммит исходников и команда загрузки соответствующего образа из Harbor по SHA-256 digest. См. [инструкцию организаторов](https://github.com/Urika000/go1-simulator-public/blob/main/docs/Инструкция_по_загрузке_решения.md).
