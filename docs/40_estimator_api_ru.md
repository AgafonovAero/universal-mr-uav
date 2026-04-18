# Estimator API Stage-1.5+

## Назначение estimator layer

`Estimator layer` добавляет минимальное code-centric оценивание состояния поверх существующего `sensor layer Stage-1.5` без изменения физической модели объекта управления.

На текущем этапе слой включает:

- оценивание ориентации по `gyro + accelerometer + magnetometer`
- оценивание высоты и вертикальной скорости по `barometer` с optional IMU prediction
- агрегатор оценивателя и runner с историей оценок

Слой намеренно остаётся прозрачным:

- без black-box toolbox решений
- без скрытого состояния вне явных struct-состояний
- без полного navigation EKF по `XYZ`

## Параметры preset

В `uav.sim.default_params_quad_x250()` добавлены baseline/demo параметры:

- `params.estimator.attitude.k_acc`
- `params.estimator.attitude.k_mag`
- `params.estimator.altitude.k_baro`
- `params.estimator.altitude.use_imu_prediction`
- `params.estimator.altitude.vz_damping`

Назначение параметров:

- `k_acc` - скорость complementary correction по `accelerometer` для `roll/pitch`, `[1/s]`
- `k_mag` - скорость complementary correction по `magnetometer` для `yaw`, `[1/s]`
- `k_baro` - скорость complementary correction по `barometer` для высоты, `[1/s]`
- `use_imu_prediction` - включает или выключает вертикальный prediction channel по `IMU specific force`
- `vz_damping` - простое демпфирование вертикальной скорости, `[1/s]`

Эти значения являются demo-baseline, а не результатом идентификации или летных испытаний.

## Сигнатуры estimator-функций

```matlab
att_est = uav.est.attitude_cf_init(params)
[att_est, diag] = uav.est.attitude_cf_step(est_prev, sens, dt_s, params)

alt_est = uav.est.altitude_filter_init(params)
[alt_est, diag] = uav.est.altitude_filter_step(est_prev, sens, att_est, dt_s, params)

est = uav.est.estimator_init(params, sens0)
[est, diag] = uav.est.estimator_step(est_prev, sens, dt_s, params)

log = uav.sim.run_case_with_estimator(case_cfg)
```

## `attitude_cf_init`

Возвращает struct начального состояния:

```matlab
att_est.q_nb
att_est.euler_rpy_rad
```

Соглашения:

- внутренняя ориентация хранится как quaternion `q_nb`
- начальная ориентация по умолчанию identity, затем корректируется измерениями
- `euler_rpy_rad` предназначены для display/debug/interface use

## `attitude_cf_step`

Возвращает:

- `att_est.q_nb`
- `att_est.euler_rpy_rad`
- `diag.quat_norm`
- `diag.acc_correction_norm`
- `diag.mag_correction_norm`

Алгоритмические соглашения:

- гироскоп используется как модель угловой скорости в `body`
- quaternion интегрируется и нормализуется на каждом шаге
- `accelerometer` используется как низкочастотная опора для `roll/pitch`
- `magnetometer` используется как низкочастотная опора для `yaw`
- коррекции выполняются явно и не зависят от скрытого состояния

Важно:

- `q_nb` поворачивает векторы из `body` в `NED`
- `Euler angles` здесь не являются основной формой хранения ориентации

## `altitude_filter_init`

Возвращает struct:

```matlab
alt_est.alt_m
alt_est.vz_mps
```

Соглашения:

- `alt_m` положительна вверх
- `vz_mps` интерпретируется как скорость изменения высоты, положительная вверх

## `altitude_filter_step`

Возвращает:

- `alt_est.alt_m`
- `alt_est.vz_mps`
- `diag.az_ned_mps2`
- `diag.baro_residual_m`

Алгоритмические соглашения:

- канал `baro.alt_m` используется как measurement correction
- при `use_imu_prediction = true`:
  - `specific force` переводится из `body` в `NED` через оцененный `q_nb`
  - затем добавляется `gravity_ned`
  - из `az_ned_mps2` строится prediction по вертикальной скорости и высоте
- используется прозрачный complementary / alpha-beta style подход
- `diag.az_ned_mps2` сохраняет знак `NED`, то есть положительное значение направлено вниз

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

- хранить явные substates слоя оценивания
- иметь короткий top-level интерфейс для runner'ов, demo и тестов

## Формат `run_case_with_estimator`

```matlab
log = uav.sim.run_case_with_estimator(case_cfg)
```

`case_cfg` использует ту же сигнатуру, что и `uav.sim.run_case`:

- `params`
- `state0`
- `dt_s`
- `t_final_s`
- `command_fun`

`run_case_with_estimator` использует:

- существующий `uav.sim.plant_step_struct`
- `uav.sensors.sensors_step`
- `uav.est.estimator_step`

Лог содержит минимум:

- `log.time_s`
- `log.state`
- `log.sensors`
- `log.estimator`
- `log.quat_norm_true`
- `log.quat_norm_est`
- `log.motor_cmd_radps`

## Единицы и знаковые соглашения

- земная система координат: `NED`
- связанная система координат: `X forward`, `Y right`, `Z down`
- внутренняя ориентация: quaternion `q_nb`
- углы в коде: только радианы
- `alt_m` положительна вверх
- `vz_mps` положительна вверх
- `diag.az_ned_mps2` положительна вниз, потому что выражена в `NED`

## Ограничения текущей стадии

- estimator layer минимальный и предназначен для demo/верификации API
- нет bias-state estimation и нет адаптации параметров
- нет полного navigation EKF по `XYZ`
- `GNSS` пока не интегрируется в объединённый навигационный оцениватель и остаётся только в sensor layer
- нет асинхронности каналов, задержек и частотных моделей измерителей
- `Euler angles` используются только как вспомогательный display/debug/interface уровень поверх quaternion state
