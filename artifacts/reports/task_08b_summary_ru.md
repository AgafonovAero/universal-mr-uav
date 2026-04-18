# TASK-08b: flight demos и plotting/reporting summary

## Какие сценарии добавлены

В репозиторий добавлены два воспроизводимых code-centric demo-сценария,
работающих поверх существующего `uav.sim.run_case_with_estimator`:

- `scripts/run_demo_takeoff_to_50m.m`
  - набор высоты до `50 m`;
  - удержание высоты;
  - сохранение `MAT` и `CSV` результатов.
- `scripts/run_demo_pitch_step_minus10deg.m`
  - набор безопасной высоты `20 m`;
  - шаг по `pitch = -10 deg`;
  - удержание режима;
  - сохранение `MAT` и `CSV` результатов.

Для наглядности добавлены отдельные plotting scripts:

- `scripts/plot_demo_takeoff_to_50m.m`
- `scripts/plot_demo_pitch_step_minus10deg.m`

Дополнительно добавлены helper-функции:

- `uav.sim.make_deterministic_demo_params`
- `uav.sim.postprocess_demo_log`
- `uav.sim.demo_series_to_table`

И smoke-level тесты:

- `tests/test_demo_takeoff_to_50m.m`
- `tests/test_demo_pitch_step_minus10deg.m`

## Что реально запускалось локально

Локально выполнены и должны быть сохранены в raw logs:

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/run_demo_takeoff_to_50m.m`
4. `scripts/run_demo_pitch_step_minus10deg.m`
5. `scripts/plot_demo_takeoff_to_50m.m`
6. `scripts/plot_demo_pitch_step_minus10deg.m`

Файлы raw logs:

- `artifacts/logs/task_08b_bootstrap.txt`
- `artifacts/logs/task_08b_runtests.txt`
- `artifacts/logs/task_08b_run_demo_takeoff_to_50m.txt`
- `artifacts/logs/task_08b_run_demo_pitch_step_minus10deg.txt`
- `artifacts/logs/task_08b_plot_demo_takeoff_to_50m.txt`
- `artifacts/logs/task_08b_plot_demo_pitch_step_minus10deg.txt`

По `artifacts/logs/task_08b_runtests.txt`:

- полный `runtests('tests')` завершился успешно;
- пройдено `45` тестов;
- `failed = 0`;
- `incomplete = 0`.

## Какие PNG создаются

После локального прогона plotting scripts формируются:

- `artifacts/figures/demo_takeoff_altitude.png`
- `artifacts/figures/demo_takeoff_vertical_speed.png`
- `artifacts/figures/demo_takeoff_motor_cmd.png`
- `artifacts/figures/demo_pitch_angle.png`
- `artifacts/figures/demo_pitch_altitude.png`
- `artifacts/figures/demo_pitch_motor_cmd.png`

## Где лежат MAT/CSV результаты

Численные результаты сохраняются в:

- `artifacts/reports/demo_takeoff_to_50m.mat`
- `artifacts/reports/demo_takeoff_to_50m.csv`
- `artifacts/reports/demo_pitch_step_minus10deg.mat`
- `artifacts/reports/demo_pitch_step_minus10deg.csv`

## Что показывают сценарии

`run_demo_takeoff_to_50m` строит честный deterministic climb/hold график:

- итоговая высота близка к `50 m`;
- пиковая ошибка по высоте остается ограниченной;
- оценка высоты из estimator layer следует истинной высоте.

По `artifacts/logs/task_08b_run_demo_takeoff_to_50m.txt`:

- final altitude: `50.002840 m`;
- peak altitude error: `3.342529 m`;
- final estimated altitude: `50.003001 m`;
- final quat norms: `true=1.000000000000`, `est=1.000000000000`.

`run_demo_pitch_step_minus10deg` строит честный pitch-step график на
безопасной высоте:

- истинный `pitch` выходит на `-10 deg`;
- высота сохраняется около безопасного уровня;
- estimator layer продолжает логироваться и попадать в MAT/CSV/PNG.

По `artifacts/logs/task_08b_run_demo_pitch_step_minus10deg.txt`:

- final pitch: `-10.000000 deg`;
- final estimated pitch: `-0.000000 deg`;
- peak pitch error: `10.000000 deg`;
- final altitude: `20.000706 m`;
- quat norms: `true=1.000000000000`, `est=1.000000000000`.

## Какие ограничения остаются

На этом шаге сохраняются ограничения:

- demos остаются script-driven и не строят GUI;
- физика объекта управления по-прежнему живет в `.m`-коде, а не в `.slx`;
- plotting scripts работают только с уже сохраненными MAT-артефактами;
- в сценарии `pitch = -10 deg` текущий estimator layer не удерживает
  оценку pitch на истинном значении в установившемся режиме, потому что
  минимальная IMU/attitude-модель еще не компенсирует steady specific-force
  эффект в наклоненном ускоренном полете.
