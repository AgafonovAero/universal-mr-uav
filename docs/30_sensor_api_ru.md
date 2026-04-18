# Sensor API Stage-1.5

## Назначение слоя датчиков

`sensor layer` добавляет code-centric измерения поверх существующего `plant kernel Stage-1.5` без изменения физической модели объекта управления. Слой не содержит скрытого состояния и не включает оцениватель состояния.

На текущем этапе доступны четыре подмодели:

- `IMU`
- `barometer`
- `magnetometer`
- `GNSS`

## Параметры preset

В `uav.sim.default_params_quad_x250()` добавлены параметры:

- `params.env.mag_ned_uT`
- `params.sensors.imu.accel_bias_b_mps2`
- `params.sensors.imu.accel_noise_std_b_mps2`
- `params.sensors.imu.gyro_bias_b_radps`
- `params.sensors.imu.gyro_noise_std_b_radps`
- `params.sensors.baro.alt_bias_m`
- `params.sensors.baro.alt_noise_std_m`
- `params.sensors.gnss.pos_bias_ned_m`
- `params.sensors.gnss.pos_noise_std_ned_m`
- `params.sensors.gnss.vel_bias_ned_mps`
- `params.sensors.gnss.vel_noise_std_ned_mps`
- `params.sensors.mag.field_bias_b_uT`
- `params.sensors.mag.field_noise_std_b_uT`

Все параметры задаются в единицах SI. Углы и угловые скорости в коде задаются в радианах и `rad/s`.

## Сигнатуры sensor-функций

```matlab
imu = uav.sensors.imu_measure(state, diag, params)
baro = uav.sensors.baro_measure(state, params)
mag = uav.sensors.mag_measure(state, params)
gnss = uav.sensors.gnss_measure(state, params)
sens = uav.sensors.sensors_step(state, diag, params)
```

### `imu_measure`

Возвращает:

- `imu.accel_b_mps2`
- `imu.gyro_b_radps`

Соглашения:

- `imu.gyro_b_radps = state.w_b_radps + bias + noise`
- `imu.accel_b_mps2` трактуется как `specific force` в связанной системе координат
- используется `diag.forces_b_N / params.mass_kg`
- гравитация второй раз не добавляется
- в level hover при нулевых шумах и `w_b = 0` ожидается примерно `[0; 0; -g]` в `body`, потому что `Z_b` направлена вниз, а положительная тяга роторов действует вдоль `-Z_b`

### `baro_measure`

Возвращает:

- `baro.alt_m`
- `baro.pressure_pa`

Соглашения:

- `baro.alt_m = -state.p_ned_m(3) + bias + noise`
- высота положительна вверх, хотя внутренняя земная система координат остаётся `NED`
- `baro.pressure_pa` вычисляется по минимальной ISA-модели тропосферы через `uav.env.isa_pressure_pa`
- отдельной модели смещения/шума давления пока нет: давление выводится из измеренной барометрической высоты

### `mag_measure`

Возвращает:

- `mag.field_b_uT`

Соглашения:

- базовое магнитное поле задаётся через `params.env.mag_ned_uT`
- `q_nb` поворачивает векторы из `body` в `NED`
- для перевода поля из `NED` в `body` используется `C_nb'`

### `gnss_measure`

Возвращает:

- `gnss.pos_ned_m`
- `gnss.vel_ned_mps`

Соглашения:

- позиция возвращается в `NED`
- скорость также возвращается в `NED`
- `state.v_b_mps` сначала переводится из связанной системы координат в `NED` через `q_nb`
- путать `body velocity` и скорость в земной системе координат нельзя

## Формат `sensors_step`

`uav.sensors.sensors_step` является тонким агрегатором без скрытого состояния и возвращает struct следующего вида:

```matlab
sens.imu.accel_b_mps2
sens.imu.gyro_b_radps
sens.baro.alt_m
sens.baro.pressure_pa
sens.mag.field_b_uT
sens.gnss.pos_ned_m
sens.gnss.vel_ned_mps
```

## Формат `run_case_with_sensors`

```matlab
log = uav.sim.run_case_with_sensors(case_cfg)
```

`case_cfg` использует ту же сигнатуру, что и `uav.sim.run_case`:

- `params`
- `state0`
- `dt_s`
- `t_final_s`
- `command_fun`

`run_case_with_sensors` использует существующий `uav.sim.plant_step_struct`, не меняет физику объекта управления и добавляет историю измерений в лог:

- `log.time_s`
- `log.state`
- `log.motor_cmd_radps`
- `log.omega_m_radps`
- `log.forces_b_N`
- `log.moments_b_Nm`
- `log.quat_norm`
- `log.sensors`

`log.sensors` является массивом struct-измерений, синхронизированным по индексам с `log.time_s`.

## Единицы и знаковые соглашения

- земная система координат: `NED`
- связанная система координат: `X forward`, `Y right`, `Z down`
- внутренняя ориентация: quaternion `q_nb`
- линейные ускорения: `m/s^2`
- угловые скорости: `rad/s`
- магнитное поле: `uT`
- высота барометра положительна вверх, хотя `p_ned_m(3)` положителен вниз

## Ограничения текущей стадии

- sensor layer пока не содержит оцениватель состояния
- нет задержек, квантизации, дискретизации по частоте и асинхронности каналов
- нет моделей насыщения или отказов датчиков
- ISA-модель атмосферы ограничена низковысотным тропосферным диапазоном, достаточным для текущих demo и unit tests
