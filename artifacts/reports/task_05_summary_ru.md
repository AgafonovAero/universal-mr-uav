# TASK-05 Summary

## Что сделано

В рамках `TASK-05` добавлен code-centric sensor layer поверх существующего `plant kernel Stage-1.5` без изменения физической модели объекта управления и без перехода к Simulink shell.

Сделано:

- расширен preset `uav.sim.default_params_quad_x250()` параметрами магнитного поля, bias и noise для `IMU`, `baro`, `GNSS`, `magnetometer`
- добавлен пакет `uav.sensors` с функциями:
  - `imu_measure`
  - `baro_measure`
  - `mag_measure`
  - `gnss_measure`
  - `sensors_step`
- добавлен runner `uav.sim.run_case_with_sensors(case_cfg)` с логированием истории измерений
- добавлена минимальная repo-local ISA-функция `uav.env.isa_pressure_pa` для расчёта барометрического давления
- добавлены demo-сценарии:
  - `scripts/run_sensor_sanity_demo.m`
  - `scripts/run_case_hover_with_sensors.m`
- добавлены unit tests:
  - `tests/test_imu_measure.m`
  - `tests/test_baro_measure.m`
  - `tests/test_mag_measure.m`
  - `tests/test_gnss_measure.m`
  - `tests/test_sensors_step.m`
  - `tests/test_run_case_with_sensors.m`
- обновлены `README.md` и `docs/30_sensor_api_ru.md`

## Что реально запускалось локально

Локально на этой машине с MATLAB были выполнены и залогированы:

- `scripts/bootstrap_project.m`
- `runtests('tests')`
- `scripts/run_sensor_sanity_demo.m`
- `scripts/run_case_hover_with_sensors.m`

Raw logs сохранены в:

- `artifacts/logs/task_05_bootstrap.txt`
- `artifacts/logs/task_05_runtests.txt`
- `artifacts/logs/task_05_sensor_sanity_demo.txt`
- `artifacts/logs/task_05_run_case_hover_with_sensors.txt`

## Результаты тестов

`runtests('tests')` выполнен успешно.

Итог:

- выполнено `23` тестовых проверки
- `23/23` passed
- `0` failed
- `0` incomplete

Новые проверки подтвердили:

- корректный deterministic `IMU` в level-hover: `gyro ~ [0; 0; 0]`, `specific force ~ [0; 0; -g]`
- корректный знак барометрической высоты и монотонное уменьшение давления с высотой
- корректный перевод магнитного поля из `NED` в `body`
- корректный перевод скорости `GNSS` из `body` в `NED`
- согласованность агрегатора `sensors_step` с одиночными sensor-функциями
- наличие sensor history и сохранение нормы кватерниона в `run_case_with_sensors`

## Результаты demo-сценариев

### `run_sensor_sanity_demo`

Получено:

- `IMU accel = [0.000000 0.000000 -9.810000] m/s^2`
- `IMU gyro = [0.000000 0.000000 0.000000] rad/s`
- `baro altitude = 25.000000 m`
- `baro pressure = 101025.031524 Pa`
- `mag body field = [20.000000 0.000000 45.000000] uT`
- `GNSS position NED = [12.000000 -3.000000 -25.000000] m`
- `GNSS velocity NED = [3.000000 -1.000000 0.500000] m/s`

### `run_case_hover_with_sensors`

Получено:

- final rotor speeds: `[553.680865 553.680865 553.680865 553.680865] rad/s`
- final specific force: `[0.000000 0.000000 -9.810000] m/s^2`
- final baro altitude: `-1.186404 m`
- final GNSS position NED: `[0.000000 0.000000 1.186404] m`
- final quaternion norm: `1.000000000000`

Небольшое снижение по высоте в hover-case ожидаемо из-за начального раскручивания роторов из нуля при наличии динамики `ESC + motor`, но не наблюдается неограниченного дрейфа.

## Ограничения текущей стадии

- sensor layer stateless и пока не моделирует частоты дискретизации каналов, задержки и асинхронность
- estimator layer отсутствует
- шум реализован как простая white-noise добавка без квантизации и saturation
- барометрическое давление считается по минимальной ISA-модели тропосферы, достаточной для текущих demo и unit tests
- нет модели отказов и деградации датчиков

## Почему следующим шагом логично делать estimator layer

Следующим инженерно логичным шагом является estimator layer, потому что:

- теперь есть стабильный и прозрачный интерфейс измерений поверх математической модели движения
- сигнатуры датчиков и формат `run_case_with_sensors` уже зафиксированы и покрыты тестами
- оцениватель можно добавлять как новый code-centric слой, не меняя физику объекта управления и не ломая plant kernel
- именно estimator layer позволит перейти от «истинного» состояния объекта управления к реалистичному контуру системы автоматического управления, который работает по измерениям `IMU`, `baro`, `GNSS` и `magnetometer`
