# TASK-09: estimator-driven flight demos и улучшение оценки ориентации

## Что было сделано

- Обновлен `uav.est.attitude_cf_step`: добавлен прозрачный
  `specific-force consistency gating` для accelerometer correction.
- В diagnostics добавлены поля `accel_correction_weight` и
  `accel_consistency_metric`.
- Сохранен простой code-centric complementary-filter подход без black-box
  EKF.
- Добавлен новый estimator-driven closed-loop runner:
  `uav.sim.run_case_closed_loop_with_estimator`.
- Добавлены demo-level controller functions:
  `uav.ctrl.demo_takeoff_hold_controller` и
  `uav.ctrl.demo_pitch_hold_controller`.
- Flight demos переведены на честное замыкание:
  `объект управления -> sensor layer -> estimator layer -> demo controller -> ВМГ`.
- Postprocessing и CSV export расширены полями
  `accel_correction_weight`, `accel_consistency_metric`,
  `pitch_estimation_error_deg` и `true_vs_est_pitch_deg`.
- Plotting scripts расширены дополнительными PNG:
  `artifacts/figures/demo_takeoff_altitude_error.png`,
  `artifacts/figures/demo_pitch_true_vs_estimated.png`,
  `artifacts/figures/demo_pitch_estimation_error.png`.
- Добавлены и обновлены unit/integration tests, включая отдельный test на
  установившийся ускоренный наклоненный полет.
- Обновлены `README.md` и `docs/40_estimator_api_ru.md`.

## Что реально запускалось локально

Обязательные локальные прогоны выполнены и сохранены как UTF-8 raw logs:

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/run_demo_takeoff_to_50m.m`
4. `scripts/run_demo_pitch_step_minus10deg.m`
5. `scripts/plot_demo_takeoff_to_50m.m`
6. `scripts/plot_demo_pitch_step_minus10deg.m`

Файлы raw logs:

- `artifacts/logs/task_09_bootstrap.txt`
- `artifacts/logs/task_09_runtests.txt`
- `artifacts/logs/task_09_run_demo_takeoff_to_50m.txt`
- `artifacts/logs/task_09_run_demo_pitch_step_minus10deg.txt`
- `artifacts/logs/task_09_plot_demo_takeoff_to_50m.txt`
- `artifacts/logs/task_09_plot_demo_pitch_step_minus10deg.txt`

По `task_09_runtests.txt`:

- `44` tests passed.
- `failed = 0`.
- `incomplete = 0`.

## Что стало лучше относительно TASK-08b

Главное улучшение относится к сценарию `pitch = -10 deg`.

- В TASK-08b итоговый true pitch был около `-10 deg`, а estimated pitch
  оставался около `0 deg`, то есть steady accelerated flight разрушал
  оценку ориентации.
- В TASK-09 accelerometer correction автоматически downweight'ится, когда
  measured `specific force` перестает быть согласованным с gravity-only
  гипотезой.
- По фактическому локальному прогону pitch-demo получены значения:
  `final true pitch = -12.121453 deg`,
  `final estimated pitch = -10.003234 deg`,
  `final pitch estimation error = -2.118220 deg`,
  `min accel correction weight after step = 0.000000`,
  `max consistency metric after step = 1.855700 m/s^2`.
- Для takeoff-demo получены значения:
  `final altitude = 49.992157 m`,
  `final estimated altitude = 49.991800 m`,
  `peak altitude error = 3.988082 m`.
- Нормы true/estimated quaternion оставались близкими к `1.0` в пределах
  численной точности.

Итог: demos действительно замыкаются на estimator outputs, а не на true
state, и estimator больше не валит `pitch` к нулю в установившемся
ускоренном наклоненном полете.

## Ограничения текущего estimator layer

- Это все еще не navigation EKF и не оцениватель траекторного движения.
- Нет оценивания bias IMU и нет адаптации параметров.
- Нет объединенного оценивания по `GNSS + IMU + baro + mag`.
- При длительном ускоренном движении без внешнего aiding остаются
  фундаментальные ограничения наблюдаемости.
- Demo controllers предназначены только для верификации и подготовки
  следующего шага, а не как production система автоматического управления.

## Почему после TASK-09 можно идти дальше к внешним flight stacks

- Интерфейс `sensor/estimator/controller` стал честнее и ближе к тому, как
  внешний flight stack реально видит объект управления.
- Demos больше не маскируют ошибку estimator layer true-state feedback'ом.
- Диагностические поля явно показывают, когда accelerometer correction
  trustworthy, а когда ее нужно downweight'ить.
- Это дает более надежную базу для следующего шага по `ArduPilot SIL`, а
  затем по `PX4 SIL`, где внешняя система автоматического управления уже
  должна опираться на measurements и estimator boundary, а не на truth.
