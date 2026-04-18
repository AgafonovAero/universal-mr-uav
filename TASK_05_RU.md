# TASK-05-RU

## Цель
Добавить code-centric sensor layer поверх существующего plant kernel Stage-1.5, не меняя физическую модель объекта управления и не переходя к Simulink shell.

Это этап перед estimator layer.

## Общие требования
- Работать ЛОКАЛЬНО на машине, где установлен MATLAB.
- Работать в новой ветке: `task/05-sensor-layer`.
- Все пояснения, commit message, PR title, PR description, summary-файлы и финальный ответ писать на русском языке.
- Не использовать GitHub cloud task для MATLAB-прогонов.
- Не создавать `.slx`, `.mlapp`, `.sldd`, `.prj`.
- Не переписывать plant kernel шире scope задачи.
- Не менять принятые системы координат и знаковые соглашения.
- Все raw logs сохранять как обычные UTF-8 text files.
- Все тесты должны быть детерминированными.

## Что нужно сделать

### 1. Расширить baseline preset параметрами среды и датчиков
Обновить:
- `src/+uav/+sim/default_params_quad_x250.m`

Нужно добавить параметры sensor layer, сохранив обратную совместимость:
- `params.env.mag_ned_uT` - вектор магнитного поля в NED, `3x1`
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

Допускается:
- noise std как scalar или `3x1`, но это должно быть явно задокументировано
- все тесты выполнять при нулевых шумах

### 2. Добавить API датчиков
Создать файлы:
- `src/+uav/+sensors/imu_measure.m`
- `src/+uav/+sensors/baro_measure.m`
- `src/+uav/+sensors/mag_measure.m`
- `src/+uav/+sensors/gnss_measure.m`
- `src/+uav/+sensors/sensors_step.m`

#### 2.1 IMU
Сигнатура:
```matlab
imu = uav.sensors.imu_measure(state, diag, params)

Выход:

imu.accel_b_mps2
imu.gyro_b_radps

Требования:

imu.gyro_b_radps = state.w_b_radps + bias + noise
imu.accel_b_mps2 трактовать как specific force в связанной системе координат
использовать диагностическую суммарную силу из plant kernel:
diag.forces_b_N / mass
не добавлять gravity второй раз
в level hover при нулевых шумах и нулевых угловых скоростях должна получаться величина, близкая к [0; 0; -g] в body frame при соглашении Z_b down
2.2 Барометр

Сигнатура:

baro = uav.sensors.baro_measure(state, params)

Выход:

baro.alt_m
baro.pressure_pa

Требования:

baro.alt_m = -state.p_ned_m(3) + bias + noise
baro.pressure_pa вычислять по ISA-модели, используя имеющуюся в репозитории атмосферную функцию, если она уже есть
2.3 Магнитометр

Сигнатура:

mag = uav.sensors.mag_measure(state, params)

Выход:

mag.field_b_uT

Требования:

использовать params.env.mag_ned_uT
переводить поле из NED в body через кватернион q_nb
если q_nb задает поворот body -> NED, то использовать согласованное преобразование в body frame
добавить bias и noise
2.4 GNSS

Сигнатура:

gnss = uav.sensors.gnss_measure(state, params)

Выход:

gnss.pos_ned_m
gnss.vel_ned_mps

Требования:

позиция берется из state.p_ned_m + bias + noise
скорость в NED вычислять из state.v_b_mps через кватернион q_nb
не путать body velocity и NED velocity
2.5 Агрегатор датчиков

Сигнатура:

sens = uav.sensors.sensors_step(state, diag, params)

Выход:

sens.imu
sens.baro
sens.mag
sens.gnss

Требования:

sensors_step должен быть тонким агрегатором без скрытого состояния
логика должна оставаться прозрачной и text-first
3. Добавить runner с датчиками

Создать файл:

src/+uav/+sim/run_case_with_sensors.m

Сигнатура:

log = uav.sim.run_case_with_sensors(case_cfg)

Требования:

использовать существующий plant_step_struct
на каждом шаге формировать sensor outputs через uav.sensors.sensors_step
не ломать существующий run_case
новый log должен содержать минимум:
time_s
state
motor_cmd_radps
omega_m_radps
forces_b_N
moments_b_Nm
quat_norm
sensors
4. Добавить demo-сценарии

Создать:

scripts/run_sensor_sanity_demo.m
scripts/run_case_hover_with_sensors.m
4.1 run_sensor_sanity_demo.m

Требования:

выполнить простой детерминированный сценарий без шумов
напечатать краткую диагностику:
IMU accel
IMU gyro
baro alt
baro pressure
mag body field
gnss position
gnss velocity
4.2 run_case_hover_with_sensors.m

Требования:

использовать uav.sim.run_case_with_sensors
моделировать hover-case
напечатать:
final rotor speeds
final specific force
final baro altitude
final gnss position
final quat norm
5. Добавить тесты

Создать:

tests/test_imu_measure.m
tests/test_baro_measure.m
tests/test_mag_measure.m
tests/test_gnss_measure.m
tests/test_sensors_step.m
tests/test_run_case_with_sensors.m

Минимально проверить:

5.1 IMU
в детерминированном level-hover:
gyro ≈ [0;0;0]
accel_b_mps2(1:2) ≈ 0
accel_b_mps2(3) ≈ -g
5.2 Барометр
baro.alt_m = -p_ned_z при нулевых bias/noise
pressure monotonically decreases with increasing altitude
5.3 Магнитометр
при единичном кватернионе значение в body совпадает с NED-полем после согласованного преобразования
выход имеет размер 3x1
5.4 GNSS
позиция имеет размер 3x1
скорость правильно переводится из body в NED
при identity attitude и нулевых bias/noise vel_ned_mps соответствует ожидаемому преобразованию
5.5 Агрегатор
sensors_step содержит все 4 подструктуры
при нулевых шумах результаты совпадают с одиночными sensor-функциями
5.6 Runner с датчиками
run_case_with_sensors возвращает log с ненулевой длиной
log содержит sensor history
норма кватерниона остается близкой к 1
в hover-case конечный baro altitude и gnss position не демонстрируют неограниченного дрейфа в детерминированном сценарии
6. Обновить документацию

Создать/обновить:

docs/30_sensor_api_ru.md
README.md

Явно задокументировать:

состав sensor layer
сигнатуры всех sensor-функций
формат sensors_step
формат run_case_with_sensors
единицы измерения
знаковые соглашения IMU / baro / GNSS / magnetometer
что sensor layer пока не включает estimator
7. Сохранить артефакты локальных прогонов

Создать и заполнить:

artifacts/logs/task_05_bootstrap.txt
artifacts/logs/task_05_runtests.txt
artifacts/logs/task_05_sensor_sanity_demo.txt
artifacts/logs/task_05_run_case_hover_with_sensors.txt
artifacts/reports/task_05_summary_ru.md

В summary указать:

что было сделано
что реально запускалось локально
результаты тестов
результаты demo-сценариев
ограничения текущей стадии
почему следующим шагом логично делать estimator layer
Ограничения
Не добавлять .slx, .mlapp, .sldd, .prj
Не переходить к estimator layer в этой задаче
Не менять controller architecture
Не усложнять модель атмосферы сверх необходимого
Не менять plant API из TASK-04-RU несовместимым образом
Не добавлять скрытое состояние в sensor layer
Критерий завершения

Задача завершена, если:

sensor layer существует как code-centric API
есть run_case_with_sensors
есть два demo-сценария
есть новые тесты
есть локальные raw logs
есть русский summary
изменения закоммичены, запушены и открыт PR