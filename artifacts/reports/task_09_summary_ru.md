# TASK-09: estimator-driven flight demos и улучшение оценки ориентации

## Что сделано

В рамках TASK-09 были выполнены четыре основных инженерных шага.

Во-первых, обновлен `uav.est.attitude_cf_step`.
В нем добавлен прозрачный `specific-force consistency gating`
для accelerometer correction.
Это позволило не тянуть оценку pitch и roll обратно к горизонту,
когда аппарат находится в установившемся ускоренном наклоненном полете.

Во-вторых, в estimator diagnostics добавлены явные поля:

- `accel_correction_weight`;
- `accel_consistency_metric`;
- `pitch_estimation_error_deg`;
- история `true_vs_est_pitch_deg`.

В-третьих, добавлен новый estimator-driven closed-loop runner
`uav.sim.run_case_closed_loop_with_estimator`.
Он замыкает demo-цепочку так:

`объект управления -> sensor layer -> estimator layer -> demo controller -> ВМГ`

В-четвертых, добавлены demo-level controller functions:

- `uav.ctrl.demo_takeoff_hold_controller`;
- `uav.ctrl.demo_pitch_hold_controller`.

Эти controller functions используют estimator outputs и не используют
true-state feedback.

Дополнительно были обновлены:

- postprocessing и CSV export;
- plotting scripts;
- unit и integration tests;
- `README.md`;
- `docs/40_estimator_api_ru.md`.

## Что запускалось

Локально были выполнены обязательные прогоны:

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/run_demo_takeoff_to_50m.m`
4. `scripts/run_demo_pitch_step_minus10deg.m`
5. `scripts/plot_demo_takeoff_to_50m.m`
6. `scripts/plot_demo_pitch_step_minus10deg.m`

Raw logs сохранены в текстовом виде:

- `artifacts/logs/task_09_bootstrap.txt`
- `artifacts/logs/task_09_runtests.txt`
- `artifacts/logs/task_09_run_demo_takeoff_to_50m.txt`
- `artifacts/logs/task_09_run_demo_pitch_step_minus10deg.txt`
- `artifacts/logs/task_09_plot_demo_takeoff_to_50m.txt`
- `artifacts/logs/task_09_plot_demo_pitch_step_minus10deg.txt`

Все эти raw logs сохранены как обычный UTF-8 text
и фиксируют реально выполненные локальные прогоны.

## Результаты тестов

По итогам `runtests('tests')`:

- `44 tests passed`
- `failed = 0`
- `incomplete = 0`

Smoke и integration tests подтвердили:

- estimator-driven closed-loop runner возвращает осмысленный log;
- нормы true и estimated quaternion остаются близкими к единице;
- takeoff-demo выходит к целевой высоте;
- pitch-demo больше не сваливает оценку pitch к нулю;
- demo controllers не используют true-state feedback.

## Результаты demo

### Takeoff demo

По локальному прогону `run_demo_takeoff_to_50m.m` получено:

- `final altitude = 49.992157 m`
- `final estimated altitude = 49.991800 m`
- `peak altitude error = 3.988082 m`

Дополнительно:

- `final accel correction weight = 1.000000`
- true и estimated quaternion norms остались близкими к `1.0`

### Pitch-step demo

По локальному прогону `run_demo_pitch_step_minus10deg.m` получено:

- `final true pitch = -12.121453 deg`
- `final estimated pitch = -10.003234 deg`
- `final pitch estimation error = -2.118220 deg`

Дополнительно:

- `min accel correction weight after step = 0.000000`
- `max consistency metric after step = 1.855700 m/s^2`
- `final altitude = 19.844794 m`
- true и estimated quaternion norms остались близкими к `1.0`

Plot scripts также были успешно выполнены локально
и построили дополнительные PNG-графики в `artifacts/figures/`.

## Улучшение относительно TASK-08b

Главное улучшение относится к установившемуся ускоренному наклоненному
полету.

В TASK-08b в сценарии `pitch = -10 deg`
истинный pitch выходил к команде,
но estimated pitch оставался около `0 deg`.
Это означало, что steady specific force ломал attitude estimator
и тянул оценку обратно к горизонту.

В TASK-09 это поведение исправлено:

- accelerometer correction автоматически downweight'ится,
  когда measured specific force перестает быть согласованным
  с gravity-only гипотезой;
- demo-контур теперь использует estimator outputs, а не true state;
- demos перестали скрывать ошибки estimator layer за счет
  true-state feedback;
- diagnostic-поля позволяют явно увидеть,
  когда accelerometer correction является надежной,
  а когда ее нужно ослабить.

Итоговый estimator-driven demo-контур стал ближе
к реальной границе внешнего flight stack
и перестал сваливать pitch к нулю в указанном режиме.

## Ограничения

- Это все еще не navigation EKF
  и не оцениватель траекторного движения.
- Нет оценивания bias IMU
  и нет адаптации параметров.
- Нет объединенного оценивания по `GNSS + IMU + baro + mag`.
- При длительном ускоренном движении без внешнего aiding
  остаются фундаментальные ограничения наблюдаемости.
- Demo controllers предназначены только для verification
  и подготовки следующего шага,
  а не как production система автоматического управления.
- TASK-09 по-прежнему не заменяет полноценный внешний flight stack
  и не выполняет навигационное слияние сенсоров уровня
  production autopilot.

## Следующий шаг

После TASK-09 можно переходить к следующему integration step
по внешним flight stack:

- текущая граница `sensor / estimator / controller`
  стала честнее;
- estimator-driven demos дают более правдоподобную verification-базу;
- `.m`-код остается source of truth,
  а `.slx` остается только thin shell;
- следующая логичная цель -
  опереться на этот слой при движении к `ArduPilot SIL`,
  а затем к `PX4 SIL`.

Практически это означает,
что после TASK-09 можно двигаться
от внутренней verification-сборки
к внешнему flight stack boundary
без возврата к true-state demo-контурам.
