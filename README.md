# Universal MR UAV

Минимальное code-centric ядро `Stage-1 / Stage-1.5` для универсального многороторного БПЛА в MATLAB. Источник истины в репозитории остаётся текстовым: `.m`-код, документы и raw logs локальных прогонов.

## Что есть в репозитории

- математическая модель движения `6DOF` на кватернионах
- простая модель ВМГ `T = kT * omega^2`, `Q = kQ * omega^2`
- геометрически корректный микшер `quad-X`
- дискретная модель `ESC + motor` первого порядка
- прозрачный `plant_step` и struct-wrapper `plant_step_struct`
- сценарные runner'ы `run_case` и `run_case_with_sensors`
- code-centric sensor layer: `IMU`, `barometer`, `magnetometer`, `GNSS`
- demo-сценарии и unit tests
- raw logs локальных MATLAB-прогонов и task summary на русском языке

## Каноническое состояние

Внешний API Stage-1.5 использует struct состояния:

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

Слой датчиков остаётся тонким и stateless. Он не меняет физическую модель объекта управления и не включает оцениватель состояния.

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

## Быстрый старт

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');
run('scripts/run_sensor_sanity_demo.m');
run('scripts/run_case_hover_with_sensors.m');
results = runtests('tests');
table(results)
```

## Архитектурная граница Stage-1.5

- реализация остаётся text-first и code-centric
- Simulink shell на этом этапе не используется
- `.slx`, `.mlapp`, `.sldd`, `.prj` не создаются
- estimator layer пока отсутствует

## Структура

```text
docs/
scripts/
src/+uav/+core/
src/+uav/+vmg/
src/+uav/+env/
src/+uav/+ctrl/
src/+uav/+sim/
src/+uav/+sensors/
tests/
artifacts/logs/
artifacts/reports/
```

См. также:

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `docs/20_plant_api_ru.md`
- `docs/30_sensor_api_ru.md`
