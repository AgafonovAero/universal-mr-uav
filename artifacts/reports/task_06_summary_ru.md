# TASK-06 Summary

## Что сделано

В рамках `TASK-06` добавлен code-centric estimator layer поверх существующего `sensor layer Stage-1.5` без изменения физической модели объекта управления и без перехода к Simulink shell.

Сделано:

- расширен preset `uav.sim.default_params_quad_x250()` параметрами estimator layer:
  - `params.estimator.attitude.k_acc`
  - `params.estimator.attitude.k_mag`
  - `params.estimator.altitude.k_baro`
  - `params.estimator.altitude.use_imu_prediction`
  - `params.estimator.altitude.vz_damping`
- добавлен пакет `uav.est` с функциями:
  - `attitude_cf_init`
  - `attitude_cf_step`
  - `altitude_filter_init`
  - `altitude_filter_step`
  - `estimator_init`
  - `estimator_step`
- добавлен runner `uav.sim.run_case_with_estimator(case_cfg)` с логированием истории датчиков и оценок
- добавлены demo-сценарии:
  - `scripts/run_estimator_sanity_demo.m`
  - `scripts/run_case_hover_with_estimator.m`
- добавлены unit tests:
  - `tests/test_attitude_cf_step.m`
  - `tests/test_altitude_filter_step.m`
  - `tests/test_estimator_step.m`
  - `tests/test_run_case_with_estimator.m`
- обновлены `README.md` и `docs/40_estimator_api_ru.md`

## Что реально запускалось локально

Локально на этой машине с MATLAB были выполнены и залогированы:

- `scripts/bootstrap_project.m`
- `runtests('tests')`
- `scripts/run_estimator_sanity_demo.m`
- `scripts/run_case_hover_with_estimator.m`

Raw logs сохранены в:

- `artifacts/logs/task_06_bootstrap.txt`
- `artifacts/logs/task_06_runtests.txt`
- `artifacts/logs/task_06_estimator_sanity_demo.txt`
- `artifacts/logs/task_06_run_case_hover_with_estimator.txt`

## Результаты тестов

`runtests('tests')` выполнен успешно.

Итог:

- выполнено `30` тестовых проверок
- `30/30` passed
- `0` failed
- `0` incomplete

Новые проверки подтвердили:

- при identity orientation и нулевой угловой скорости attitude estimator сохраняет ориентацию
- норма оцененного quaternion остаётся близкой к `1`
- в level-hover при нулевых шумах `roll/pitch` оцениваются близко к нулю
- при постоянной baro altitude altitude estimate сходится к барометрическому каналу
- при нулевом вертикальном ускорении `vz_mps` не расходится
- агрегатор `estimator_step` возвращает обязательные поля `q_nb`, `euler_rpy_rad`, `alt_m`, `vz_mps`
- `run_case_with_estimator` возвращает history оценивателя и не демонстрирует неограниченного дрейфа оценки высоты в детерминированном hover-case

## Результаты demo-сценариев

### `run_estimator_sanity_demo`

Получено:

- `estimated roll = 0.000000 rad`
- `estimated pitch = 0.000000 rad`
- `estimated yaw = 0.261718 rad`
- `estimated altitude = 12.000000 m`
- `estimated vertical speed = 0.000000 m/s`
- `quaternion norm = 1.000000000000`

### `run_case_hover_with_estimator`

Получено:

- `final estimated Euler = [0.000000 0.000000 0.000000] rad`
- `final estimated altitude = -1.159642 m`
- `final estimated vertical speed = -0.528675 m/s`
- `final estimated quaternion norm = 1.000000000000`

Отрицательная высота в hover-case на текущем demo ожидаема, потому что модель запускается с нулевых оборотов роторов и проходит фазу раскрутки `ESC + motor` до hover-команды. При этом estimator остаётся согласованным с sensor layer и не показывает неограниченного дрейфа.

## Ограничения текущей стадии

- estimator layer минимальный и предназначен для прозрачной верификации интерфейсов
- нет bias-state estimation, adaptive tuning и полного navigation EKF по `XYZ`
- `GNSS` пока не интегрируется в объединённый навигационный оцениватель и остаётся только в sensor layer
- нет асинхронности каналов, частотных моделей, задержек, saturation и fault-моделей
- `Euler angles` используются только как display/debug/interface слой поверх quaternion state

## Почему следующим шагом логично делать thin Simulink shell или navigation EKF

Следующим инженерно логичным шагом теперь можно выбирать один из двух путей:

- `thin Simulink shell`, если нужен orchestration-уровень поверх уже стабилизированных code-centric слоёв `plant + sensors + estimator` без переноса физической логики в бинарные артефакты
- `navigation EKF`, если следующая цель — начать объединять `GNSS`, `baro`, `IMU` и `magnetometer` в более полный навигационный оцениватель после того, как API локального estimator layer уже зафиксирован тестами и demo-сценариями

Оба пути теперь опираются на явные struct-интерфейсы, raw logs локальных прогонов и существующую верификацию.
