# Estimator API Stage-1.5+

## Назначение estimator layer

`Estimator layer` добавляет минимальное code-centric оценивание состояния
поверх существующего `sensor layer Stage-1.5+` без переноса логики в
`Simulink` и без изменения физики объекта управления.

На текущем этапе слой включает:

- оценивание ориентации по `gyro + accelerometer + magnetometer`;
- оценивание высоты и вертикальной скорости по `barometer` с optional
  prediction по `IMU specific force`;
- агрегатор оценивателя для runner'ов, thin MIL/SIL shell и demo-сценариев.

Слой намеренно остается прозрачным:

- без black-box EKF;
- без скрытого состояния вне явных struct-состояний;
- без переноса физики математической модели движения в `.slx`.

## Параметры preset

В `uav.sim.default_params_quad_x250()` используются baseline/demo
параметры:

- `params.estimator.attitude.k_acc`
- `params.estimator.attitude.k_mag`
- `params.estimator.attitude.accel_consistency_full_weight_mps2`
- `params.estimator.attitude.accel_consistency_zero_weight_mps2`
- `params.estimator.altitude.k_baro`
- `params.estimator.altitude.use_imu_prediction`
- `params.estimator.altitude.vz_damping`

Назначение новых attitude-параметров:

- `k_acc` - скорость complementary correction по accelerometer для
  `roll/pitch`, `[1/s]`;
- `k_mag` - скорость complementary correction по magnetometer для `yaw`,
  `[1/s]`;
- `accel_consistency_full_weight_mps2` - порог residual specific force, до
  которого accelerometer correction имеет полный вес;
- `accel_consistency_zero_weight_mps2` - порог residual specific force, от
  которого accelerometer correction полностью отключается.

Идея gating простая: estimator сравнивает измеренный `specific force` в
связанной системе координат с ожидаемым gravity-only specific force,
полученным из текущего quaternion prediction. Если residual мал, коррекция
по accelerometer работает как раньше. Если residual велик, коррекция
плавно подавляется, чтобы установившийся ускоренный наклоненный полет не
тянул оценку `pitch/roll` обратно к нулю.

## Сигнатуры estimator-функций

```matlab
att_est = uav.est.attitude_cf_init(params)
[att_est, diag] = uav.est.attitude_cf_step(est_prev, sens, dt_s, params)

alt_est = uav.est.altitude_filter_init(params)
[alt_est, diag] = uav.est.altitude_filter_step(est_prev, sens, att_est, dt_s, params)

est = uav.est.estimator_init(params, sens0)
[est, diag] = uav.est.estimator_step(est_prev, sens, dt_s, params)

log = uav.sim.run_case_with_estimator(case_cfg)
log = uav.sim.run_case_closed_loop_with_estimator(case_cfg)
```

## `attitude_cf_step`

Возвращает:

- `att_est.q_nb`
- `att_est.euler_rpy_rad`
- `diag.quat_norm`
- `diag.accel_correction_weight`
- `diag.accel_consistency_metric`
- `diag.acc_correction_norm`
- `diag.mag_correction_norm`

Алгоритмические соглашения:

- гироскоп используется как prediction model по угловой скорости в
  связанной системе координат;
- quaternion интегрируется и нормализуется на каждом шаге;
- accelerometer используется как низкочастотная опора для `roll/pitch`
  только тогда, когда measured specific force согласован с gravity-only
  гипотезой;
- magnetometer используется как низкочастотная опора для `yaw`;
- коррекции выполняются явно и не зависят от скрытого состояния.

Важно:

- `q_nb` поворачивает векторы из связанной системы координат в земную
  систему координат `NED`;
- `Euler angles` остаются только display/debug/interface representation
  поверх quaternion.

## `altitude_filter_step`

Возвращает:

- `alt_est.alt_m`
- `alt_est.vz_mps`
- `diag.az_ned_mps2`
- `diag.baro_residual_m`

Соглашения:

- `alt_m` положительна вверх;
- `vz_mps` положительна вверх;
- `diag.az_ned_mps2` хранится в знаке `NED`, то есть положительное
  значение направлено вниз.

## `estimator_init` и `estimator_step`

Агрегатор возвращает единый estimator state:

```matlab
est.attitude
est.altitude
est.q_nb
est.euler_rpy_rad
est.alt_m
est.vz_mps
```

`diag` агрегатора содержит:

```matlab
diag.attitude
diag.altitude
```

Это позволяет:

- хранить явные substates estimator layer;
- иметь короткий top-level интерфейс для runner'ов, demo и thin shell.

## `run_case_with_estimator`

`uav.sim.run_case_with_estimator(case_cfg)` остается базовым transparent
runner'ом для сценариев вида:

`объект управления -> sensor layer -> estimator layer`

Он полезен для отладки estimator и совместимости с существующими
TASK-07/TASK-08 сценариями.

## `run_case_closed_loop_with_estimator`

`uav.sim.run_case_closed_loop_with_estimator(case_cfg)` добавляет явную
цепочку:

`объект управления -> sensor layer -> estimator layer -> demo controller -> ВМГ -> объект управления`

`case_cfg` содержит:

- `params`
- `state0`
- `dt_s`
- `t_final_s`
- `controller_fun`
- optional `reference_fun`
- optional `controller_state0`

Лог нового runner'а содержит минимум:

- `log.time_s`
- `log.state`
- `log.sensors`
- `log.estimator`
- `log.reference`
- `log.estimator_diag`
- `log.controller_diag`
- `log.controller_state`
- `log.quat_norm_true`
- `log.quat_norm_est`
- `log.motor_cmd_radps`

Этот runner нужен именно для estimator-driven verification и подготовки к
следующему шагу по внешним flight stacks.

## Единицы и знаковые соглашения

- земная система координат: `NED`;
- связанная система координат: `X forward`, `Y right`, `Z down`;
- внутренняя ориентация: quaternion `q_nb`;
- углы в коде: только `rad`;
- `alt_m` и `vz_mps` положительны вверх;
- `specific force` в IMU хранится в связанной системе координат.

## Ограничения текущей стадии

- estimator layer остается минимальным и ориентированным на верификацию;
- нет bias-state estimation и полного navigation EKF;
- нет интеграции `GNSS` в объединенный навигационный оцениватель;
- нет асинхронности каналов, задержек и частотных моделей измерителей;
- specific-force gating улучшает оценку ориентации в ускоренном
  наклоненном полете, но не заменяет полноценный оцениватель траекторного
  движения для будущей внешней системы автоматического управления.
