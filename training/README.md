# Дообучение Go1

Исходная политика — Walk These Ways, эксперимент `pretrain-v0`, ревизия
`0e7236bdc81ce855cbe3d70345a7899452bdeb1c`. Лицензия находится в корне
репозитория. [Веса авторов](https://github.com/Improbable-AI/walk-these-ways/tree/0e7236bdc81ce855cbe3d70345a7899452bdeb1c/runs/gait-conditioned-agility/pretrain-v0/train/025417.456545/checkpoints)

Обновлялись веса и смещения последнего линейного слоя актёра. Остальные
слои актёра и модуль адаптации были заморожены. PPO собрал 13 440 шагов
в MuJoCo и выполнил 440 обновлений. В обучении и локальной оценке работал
Safety Block из Т2. Для расчёта награды использовались линейная скорость,
ориентация и высота корпуса в обучающей среде. Входы ROS-политики остаются
`/lowstate` и `/cmd_vel`, её такт задаёт `/clock`.

Конфигурация — [ppo.json](ppo.json), журнал — [ppo.jsonl](results/ppo.jsonl).
Выбран чекпойнт итерации 10 по награде на двух проверочных эпизодах.
Итоговые веса — [t3_weights.npz](../t3_weights.npz).

| Файл | SHA-256 |
| --- | --- |
| `adaptation_module_latest.jit` | `f0adb113954058a37d2596282f041a3bb128365558ecf64405bbc1353d5dc1b0` |
| `body_latest.jit` | `6c14b59ca28550a012de3bf5d21fc7d6aa6b6d38eab4dc31e172519f12a19ece` |
| `baseline.npz` после экспорта | `a003991b156e2170bddc6d9e264c595a39421a9cf0215a2f031d7e1e0f616a26` |
| `t3_weights.npz` | `77e5e383b0db8212626968e49d499cdeda06bf9b67bbf1038656b8c1e42fd336` |

## Запуск обучения

Команды выполняются из корня репозитория. Для обучения использовались
Python 3.12, CPU-версия PyTorch 2.6.0, MuJoCo 3.3.7 и NumPy 2.5.5.
Путь к модели должен указывать на `models/mujoco/unitree_go1` из симулятора
версии `abce1d8afe39e72f5287318e0e538b1cdfd909a8`.

```bash
python3.12 -m venv ~/.venvs/go1-training
source ~/.venvs/go1-training/bin/activate
python -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.6.0+cpu'
python -m pip install 'mujoco==3.3.7' 'numpy==2.5.5'
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
work=$(mktemp -d)
model=~/go1-simulator-public/models/mujoco/unitree_go1
src=https://raw.githubusercontent.com/Improbable-AI/walk-these-ways/0e7236bdc81ce855cbe3d70345a7899452bdeb1c/runs/gait-conditioned-agility/pretrain-v0/train/025417.456545/checkpoints
curl -fsSL "$src/adaptation_module_latest.jit" -o "$work/adaptation_module_latest.jit"
curl -fsSL "$src/body_latest.jit" -o "$work/body_latest.jit"
python -m training.export_wtw "$work" "$work/baseline.npz"
python -m training.finetune "$model" "$work/baseline.npz" training/ppo.json "$work/ppo"
python -m training.evaluate "$model" "$work/baseline.npz" --episodes 10 --seed 4700
python -m training.evaluate "$model" "$work/ppo/best.npz" --episodes 10 --seed 4700
```

## Контрольные эпизоды

Файлы [baseline.jsonl](results/baseline.jsonl) и
[finetuned.jsonl](results/finetuned.jsonl) содержат десять измерений для каждой
политики при команде 0,4 м/с. Для обеих серий использовались одинаковые
начальные отклонения суставов с генераторами 4700–4709. Значение
`sim_seconds` включает первые 1,5 секунды прогрева.

| Политика | Успехи | Среднее зачётное время на 3 м |
| --- | ---: | ---: |
| Исходная | 10/10 | 23,53 с |
| Дообученная | 10/10 | 22,83 с |

[ros-mujoco.jsonl](results/ros-mujoco.jsonl) содержит отдельную серию с
итоговыми весами. Для каждого прогона заново запускались контейнеры участника
и публичного симулятора. Команда скорости поступала через ROS, измерение
проводилось в отдельном тестовом процессе. Успешны 10 из 10 эпизодов,
время после прогрева — от 23,25 до 25,51 с.
