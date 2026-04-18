# Universal MR UAV

Минимальное code-centric ядро `Stage-1.5+` для универсального многороторного БПЛА в MATLAB/Simulink. Источник истины в репозитории остаётся текстовым: `.m`-код, документы и raw logs локальных прогонов. Simulink на текущем этапе разрешён только как thin orchestration shell поверх уже существующего `.m`-ядра.

## Что есть в репозитории

- математическая модель движения `6DOF` на кватернионах
- простая модель ВМГ `T = kT * omega^2`, `Q = kQ * omega^2`
- геометрически корректный микшер `quad-X`
- дискретная модель `ESC + motor` первого порядка
- прозрачный `plant_step`, `plant_step_struct` и сценарные runner'ы
- code-centric sensor layer: `IMU`, `barometer`, `magnetometer`, `GNSS`
- минимальный estimator layer:
  - `attitude complementary filter` по `gyro + accel + mag`
  - `altitude complementary filter` по `baro` с optional IMU prediction
- thin Simulink MIL shell:
  - `uav.sl.Stage15MILSystem`
  - script-generated `models/mil_top.slx`
  - bus definitions без `.sldd`
- demo-сценарии и unit tests
- raw logs локальных MATLAB-прогонов и task summary на русском языке

## Каноническое состояние объекта управления

Внешний API использует struct состояния:

```text
state.p_ned_m
state.v_b_mps
state.q_nb
state.w_b_radps
state.omega_m_radps
```

Где:

- `p_ned_m` - положение в земной системе координат `NED`
- `v_b_mps` - линейная скорость в связанной системе координат
- `q_nb` - scalar-first quaternion поворота из `body` в `NED`
- `w_b_radps` - угловая скорость в связанной системе координат
- `omega_m_radps` - угловые скорости четырёх роторов

Packed-представление для совместимости с kernel:

```text
x_plant = [p_ned_m(3); v_b_mps(3); q_nb(4); w_b_radps(3); omega_m_radps(4)]
```

## Sensor Layer Stage-1.5

Слой датчиков остаётся тонким и stateless. Он не меняет физическую модель объекта управления и не содержит скрытого состояния.

Доступные функции:

```matlab
imu = uav.sensors.imu_measure(state, diag, params);
baro = uav.sensors.baro_measure(state, params);
mag = uav.sensors.mag_measure(state, params);
gnss = uav.sensors.gnss_measure(state, params);
sens = uav.sensors.sensors_step(state, diag, params);
log = uav.sim.run_case_with_sensors(case_cfg);
```

Ключевые соглашения:

- `IMU accel` - `specific force` в `body`
- `baro.alt_m = -p_ned_z + bias + noise`
- `GNSS` возвращает позицию и скорость в `NED`
- `magnetometer` использует `params.env.mag_ned_uT` и `q_nb`
- все углы в коде задаются в радианах

Подробности: `docs/30_sensor_api_ru.md`

## Estimator Layer

Estimator layer добавляет прозрачное оценивание состояния поверх sensor layer, не меняя физическую модель объекта управления.

Доступные функции:

```matlab
attitude = uav.est.attitude_cf_init(params);
[attitude, att_diag] = uav.est.attitude_cf_step(attitude, sens, dt_s, params);

altitude = uav.est.altitude_filter_init(params);
[altitude, alt_diag] = uav.est.altitude_filter_step(altitude, sens, attitude, dt_s, params);

est = uav.est.estimator_init(params, sens0);
[est, est_diag] = uav.est.estimator_step(est, sens, dt_s, params);

log = uav.sim.run_case_with_estimator(case_cfg);
```

Итоговый estimator output содержит минимум:

```matlab
est.q_nb
est.euler_rpy_rad
est.alt_m
est.vz_mps
```

Соглашения estimator layer:

- внутренняя ориентация хранится в quaternion `q_nb`
- `accelerometer` корректирует `roll/pitch`
- `magnetometer` корректирует `yaw`
- `alt_m` положительна вверх
- `vz_mps` интерпретируется как вертикальная скорость по высоте, положительная вверх
- `GNSS` пока не входит в navigation EKF и остаётся только в sensor layer

Подробности: `docs/40_estimator_api_ru.md`

## Thin Simulink Shell Stage-1.5+

`Simulink shell` на текущем этапе разрешён, но только в виде минимальной MIL-orchestration оболочки над существующим code-centric ядром.

Состав thin shell:

- `uav.sl.Stage15MILSystem`:
  - принимает `motor_cmd_radps`
  - внутри вызывает `uav.sim.plant_step_struct`
  - затем формирует `uav.sensors.sensors_step`
  - затем продвигает `uav.est.estimator_step`
- `uav.sl.make_bus_defs` создаёт `Simulink.Bus` definitions в base workspace
- `uav.sl.make_demo_command_profile` формирует дискретные профили команд на моторы
- `scripts/build_mil_top.m` программно создаёт или перестраивает `models/mil_top.slx`

Почему shell считается thin:

- физика объекта управления остаётся в `src/+uav/+sim`, `src/+uav/+core`, `src/+uav/+vmg`
- sensor layer остаётся в `src/+uav/+sensors`
- estimator layer остаётся в `src/+uav/+est`
- `.slx` не является source of truth и регенерируется из `.m`-скрипта

Подробности: `docs/50_simulink_shell_ru.md`

## Быстрый старт

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');
run('scripts/run_sensor_sanity_demo.m');
run('scripts/run_case_hover_with_sensors.m');
run('scripts/run_estimator_sanity_demo.m');
run('scripts/run_case_hover_with_estimator.m');
run('scripts/build_mil_top.m');
run('scripts/run_mil_top_hover.m');
run('scripts/run_mil_top_yaw_step.m');
results = runtests('tests');
table(results)
```

## Архитектурная граница текущего этапа

- реализация остаётся text-first и code-centric
- sensor layer и estimator layer уже являются частью code-centric ядра
- Simulink shell разрешён только как thin orchestration layer
- `.slx` допускается только в минимальном виде и не становится source of truth
- `.mlapp`, `.sldd`, `.prj` не создаются
- estimator layer минимальный и прозрачный: без полного navigation EKF по `XYZ`
- `GNSS` остаётся источником измерений sensor layer и пока не интегрируется в объединённый навигационный оцениватель
- нет ручной блочной реализации физики объекта, sensor layer или estimator layer внутри `.slx`

## Структура

```text
docs/
scripts/
src/+uav/+core/
src/+uav/+vmg/
src/+uav/+env/
src/+uav/+ctrl/
src/+uav/+sensors/
src/+uav/+est/
src/+uav/+sim/
src/+uav/+sl/
models/
tests/
artifacts/logs/
artifacts/reports/
```

См. также:

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `docs/20_plant_api_ru.md`
- `docs/30_sensor_api_ru.md`
- `docs/40_estimator_api_ru.md`
- `docs/50_simulink_shell_ru.md`
