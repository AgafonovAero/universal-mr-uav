# Отчет по TASK-04-RU

## Что сделано

- Зафиксирован канонический API состояния plant kernel:
  - `uav.core.state_validate`
  - `uav.core.state_pack`
  - `uav.core.state_unpack`
- Добавлен struct-wrapper `uav.sim.plant_step_struct` поверх существующего `uav.sim.plant_step`.
- Добавлен единый сценарный runner `uav.sim.run_case` с конфигурацией через `case_cfg`.
- Добавлены два demo-сценария на новом runner:
  - `scripts/run_case_hover.m`
  - `scripts/run_case_yaw_step.m`
- Добавлены четыре новых теста:
  - `tests/test_state_pack_unpack.m`
  - `tests/test_plant_step_struct.m`
  - `tests/test_run_case_hover.m`
  - `tests/test_run_case_yaw_step.m`
- Обновлены `README.md`, `scripts/bootstrap_project.m` и создана документация `docs/20_plant_api_ru.md`.

## Что реально запускалось локально

Дата локального прогона: `2026-04-18`

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/run_case_hover.m`
4. `scripts/run_case_yaw_step.m`

Raw logs:

- `artifacts/logs/task_04_bootstrap.txt`
- `artifacts/logs/task_04_runtests.txt`
- `artifacts/logs/task_04_run_case_hover.txt`
- `artifacts/logs/task_04_run_case_yaw_step.txt`

## Результаты тестов

- Локально выполнены все тесты из каталога `tests`.
- Всего прошло `16` тестов, падений и incomplete-результатов не было.
- Новые проверки TASK-04 подтвердили:
  - обратимость `state_pack/state_unpack`
  - отлов ошибки размерности в `state_validate`
  - согласованность `plant_step_struct` с `plant_step`
  - сохранение нормы кватерниона
  - близкое к нулю вертикальное ускорение в hover-case после раскрутки
  - положительный знак `w_z` в yaw-step case при положительном yaw moment

## Результаты demo-сценариев

### run_case_hover

- `hover command = [553.681 553.681 553.681 553.681] rad/s`
- `final rotor speeds = [553.681 553.681 553.681 553.681] rad/s`
- `total thrust = 9.810000 N`
- `weight = 9.810000 N`
- `vertical accel estimate = 7.105427e-15 m/s^2`
- `final quat norm = 1.000000000000`

### run_case_yaw_step

- `step time = 0.600 s`
- `final yaw moment = 0.020000 N*m`
- `final yaw rate = 3.022337 rad/s`
- `final rotor speeds = [590.109 514.680 590.109 514.680] rad/s`
- `total thrust = 9.810000 N`
- `hover weight = 9.810000 N`
- `final quat norm = 1.000000000000`

## Ограничения текущей стадии

- Работа по-прежнему ограничена `Stage-1 / Stage-1.5`.
- `.slx`, `.mlapp`, `.sldd`, `.prj` не создавались.
- Sensor model, estimator layer и Simulink shell не добавлялись.
- Интегратор остается простым explicit Euler без усложнения схемы.
- Система автоматического управления и архитектура controller layer не переделывались.

## Что логично делать дальше

- Добавить sensor layer поверх канонического состояния plant kernel.
- Ввести estimator layer без изменения принятого внешнего API состояния.
- После этого строить thin Simulink shell как оркестрационный слой над code-centric ядром.
