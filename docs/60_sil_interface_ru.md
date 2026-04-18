# SIL Interface Layer Stage-1.5+

## Назначение документа

Этот документ фиксирует подготовительный SIL interface layer, добавленный в TASK-08.

Главная цель шага:

- подготовить репозиторий к приоритетному подключению внешних flight stack'ов
- не разрушить code-centric архитектуру
- не превратить `.slx` в source of truth

Приоритет roadmap после TASK-08:

1. ArduPilot SIL
2. PX4 SIL
3. PX4 HIL

## Что входит в TASK-08

TASK-08 делает только SIL-prep boundary:

- канонический внешний actuator packet
- канонический внешний sensor packet
- прозрачный multi-rate scheduler на packet boundary
- `Stage15SILBridgeSystem` как thin wrapper над существующим ядром
- `StubExternalFCSSystem` как временный placeholder-controller для smoke tests
- script-generated `models/sil_top.slx`

TASK-08 не делает:

- реальную интеграцию ArduPilot
- реальную интеграцию PX4
- runtime bridge по MAVLink/UDP
- HIL

## Главный архитектурный принцип

Источник истины остается в:

- `.m`-коде
- текстовых документах
- raw logs локальных прогонов

Simulink допускается только как thin shell, который:

- создается скриптом
- использует существующее `.m`-ядро
- не переносит plant/sensor/estimator logic внутрь block diagram

## Пакет `uav.sil`

TASK-08 добавляет пакет:

```text
src/+uav/+sil/
```

Он фиксирует code-centric внешний интерфейс для будущих стек-специфичных adapter'ов.

### `uav.sil.actuator_cmd_to_motor_radps`

Назначение:

- преобразовать внешний actuator packet в `motor_cmd_radps`

Поддерживаемый канонический формат:

```matlab
act.mode
act.motor_norm_01
```

На текущем этапе:

- поддерживается только `mode = "norm01"`
- `motor_norm_01` имеет размер `4x1`
- диапазон нормированной команды saturates к `[0, 1]`
- преобразование в моторные скорости использует только:
  - `params.motor.omega_min_radps`
  - `params.motor.omega_max_radps`

### `uav.sil.hover_trim_from_params`

Назначение:

- вычислить номинальный hover trim в нормированной форме

Использование:

- stub-controller
- smoke tests
- deterministic initial actuator conditions

### `uav.sil.make_sensor_packet`

Назначение:

- упаковать выходы sensor layer в канонический внешний пакет

Минимальный состав пакета:

```matlab
packet.time_s
packet.imu_valid
packet.imu.accel_b_mps2
packet.imu.gyro_b_radps
packet.baro_valid
packet.baro.alt_m
packet.baro.pressure_pa
packet.mag_valid
packet.mag.field_b_uT
packet.gnss_valid
packet.gnss.pos_ned_m
packet.gnss.vel_ned_mps
```

Семантика valid-флагов:

- `true` означает, что на текущем base step появился новый sample данного канала
- значения медленных датчиков между обновлениями удерживаются по zero-order hold

### `uav.sil.update_rate_scheduler`

Назначение:

- реализовать детерминированный multi-rate scheduler для packet boundary

Схема rate groups:

- `IMU`: every base step
- `baro`: каждые `0.02 s`
- `mag`: каждые `0.02 s`
- `GNSS`: каждые `0.1 s`

Реализация:

- без глобального состояния
- с явным struct-состоянием scheduler'а
- через discrete counters / decimation logic

## `Stage15SILBridgeSystem`

`uav.sl.Stage15SILBridgeSystem` является ключевой boundary-точкой TASK-08.

Он:

- принимает внешний actuator command struct
- хранит внутреннее состояние объекта управления
- хранит внутреннее состояние estimator layer
- вызывает существующие `.m` API:
  - `uav.sim.plant_step_struct`
  - `uav.sensors.sensors_step`
  - `uav.est.estimator_step`
  - `uav.sil.*`

Он не делает ручной re-implementation plant/sensor/estimator logic внутри Simulink.

### Выходы bridge

`sensor_packet_out`:

- канонический внешний sensor packet

`truth_out`:

- `truth_out.state`
- `truth_out.estimator`

`diag_out`:

- `quat_norm_true`
- `quat_norm_est`
- `omega_m_radps`

## `StubExternalFCSSystem`

`uav.sl.StubExternalFCSSystem` не является реальным автопилотом.

Это временный placeholder для smoke tests, который:

- принимает внешний sensor packet
- выдает внешний actuator command
- умеет два режима:
  - `hover`
  - `yaw_step`

Контур intentionally minimal:

- hover trim из параметров
- небольшая baro-based collective correction
- небольшой differential yaw bias для режима `yaw_step`

Этот stub нужен только для демонстрации замкнутой boundary и позже должен быть заменен стек-специфичными adapter'ами.

## `make_sil_bus_defs`

`uav.sl.make_sil_bus_defs` создает `Simulink.Bus` objects без `.sldd` для:

- external actuator command
- external sensor packet
- truth output
- diag output

Bus definitions соответствуют именно каноническому API TASK-08 и поддерживаются как base-workspace артефакты.

## `sil_top.slx`

`models/sil_top.slx` создается только через:

```matlab
run('scripts/build_sil_top.m');
```

В модели остаются только минимально необходимые orchestration-блоки:

- `StubExternalFCSSystem`
- `Stage15SILBridgeSystem`
- один минимальный feedback delay на packet feedback path
- логирование основных выходов

В модели нет ручной block-diagram реализации:

- физики объекта управления
- sensor layer
- estimator layer

## Почему этот шаг является именно SIL-prep

TASK-08 считается подготовкой к ArduPilot/PX4 SIL, потому что он уже фиксирует:

- форму actuator boundary
- форму sensor boundary
- минимальный scheduler boundary
- отдельный Simulink shell для будущего внешнего flight stack

Но при этом он еще не привязан к конкретному стеку и не вводит transport/runtime coupling.

## Почему реальная интеграция вынесена в следующие задачи

Настоящая интеграция стеков требует отдельных решений по:

- transport/runtime contract
- стек-специфичному mapping'у actuator outputs
- стек-специфичному packaging sensor data
- режимам запуска, orchestration и верификации

После TASK-08 эти вопросы можно решать уже как отдельные стек-специфичные задачи, не переделывая базовую boundary-архитектуру.

## Почему следующим шагом логично делать именно ArduPilot SIL

Потому что к этому моменту уже готовы:

- канонический внешний actuator interface
- канонический внешний sensor interface
- script-generated thin SIL shell
- smoke-test boundary

Значит следующий инженерный шаг должен не расширять abstract shell еще сильнее, а впервые подключить реальный внешний stack поверх уже зафиксированной boundary. По roadmap первым таким шагом должен стать именно ArduPilot SIL.
