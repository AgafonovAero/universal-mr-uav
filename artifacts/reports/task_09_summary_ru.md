# TASK-09: estimator-driven flight demos и улучшение оценки ориентации

## Что было сделано

- обновлен `uav.est.attitude_cf_step`:
  - добавлен прозрачный `specific-force consistency gating` для
    accelerometer correction;
  - добавлены diagnostics:
    - `accel_correction_weight`
    - `accel_consistency_metric`;
- сохранен простой code-centric complementary-filter подход без black-box
  EKF;
- добавлен новый estimator-driven closed-loop runner:
  - `uav.sim.run_case_closed_loop_with_estimator`;
- добавлены demo-level controller functions:
  - `uav.ctrl.demo_takeoff_hold_controller`
  - `uav.ctrl.demo_pitch_hold_controller`;
- flight demos переведены на честное замыкание:
  - `объект управления -> sensor layer -> estimator layer -> demo controller -> ВМГ`;
- postprocessing и CSV export расширены новыми полями:
  - `accel_correction_weight`
  - `accel_consistency_metric`
  - `pitch_estimation_error_deg`
  - `true_vs_est_pitch_deg`;
- plotting scripts расширены дополнительными PNG:
  - `artifacts/figures/demo_takeoff_altitude_error.png`
  - `artifacts/figures/demo_pitch_true_vs_estimated.png`
  - `artifacts/figures/demo_pitch_estimation_error.png`;
- добавлены/обновлены unit и integration tests, включая отдельный test на
  установившийся ускоренный наклоненный полет;
- обновлены `README.md` и `docs/40_estimator_api_ru.md`.

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

- `44` tests passed;
- `failed = 0`;
- `incomplete = 0`.

## Что стало лучше относительно TASK-08b

Главное улучшение касается сценария `pitch = -10 deg`:

- в TASK-08b итоговый true pitch был около `-10 deg`, а estimated pitch
  оставался около `0 deg`, то есть steady accelerated flight разрушал
  оценку ориентации;
- в TASK-09 для этого же класса сценариев accelerometer correction
  автоматически downweight'ится, когда measured `specific force`
  перестает быть согласованным с gravity-only гипотезой;
- по фактическому локальному прогону:
  - final true pitch: `-12.121453 deg`
  - final estimated pitch: `-10.003234 deg`
  - final pitch estimation error: `-2.118220 deg`
  - min accel correction weight after step: `0.000000`
  - max consistency metric after step: `1.855700 m/s^2`

Для takeoff-demo:

- final altitude: `49.992157 m`
- final estimated altitude: `49.991800 m`
- peak altitude error: `3.988082 m`
- quaternion norms true/est remained `1.0` within numerical tolerance.

Итог: demos теперь действительно замыкаются на estimator outputs, а не на
true state, и estimator перестал валить `pitch` к нулю в установившемся
ускоренном наклоненном полете.

## Ограничения текущего estimator layer

- это все еще не navigation EKF и не оцениватель траекторного движения;
- нет оценивания bias IMU и нет адаптации параметров;
- нет объединенного оценивания по `GNSS + IMU + baro + mag`;
- при длительном ускоренном движении без внешнего aiding остаются
  фундаментальные ограничения наблюдаемости;
- demo controllers предназначены только для верификации и подготовки
  следующего шага, а не как production система автоматического управления.

## Почему после TASK-09 можно идти дальше к внешним flight stacks

- интерфейс `sensor/estimator/controller` стал честнее и ближе к тому, как
  внешний flight stack реально видит объект управления;
- demos больше не маскируют ошибку estimator layer true-state feedback'ом;
- диагностические поля теперь явно показывают, когда accelerometer
  correction trustworthy, а когда ее нужно downweight'ить;
- это дает более надежную базу для следующего шага по `ArduPilot SIL` и
  затем `PX4 SIL`, где внешняя система автоматического управления уже
  должна опираться на measurements и estimator boundary, а не на truth.
