# ArduPilot JSON SITL adapter scaffold

## Назначение

TASK-10 подготавливает code-centric adapter layer для будущей связи
`universal-mr-uav` с `ArduPilot SITL` через JSON interface.

На этом шаге не требуется:

- поднять реальный ArduPilot SITL;
- добиться полноценного полета;
- реализовать production-ready UDP bridge;
- переносить физическую модель в `.slx`.

Цель TASK-10 уже уже другая:

- зафиксировать интерфейс boundary;
- сделать прозрачные pack/unpack functions;
- сделать PWM -> motor command mapping;
- сделать loopback smoke test без реального ArduPilot;
- документировать текущие ограничения и следующий шаг.

## Почему нужен отдельный adapter layer

ArduPilot JSON SITL работает как внешний physics backend через UDP.

Чтобы в следующем шаге MATLAB/Simulink мог выступать таким backend,
нужно заранее явно согласовать:

- состав FDM/state packet;
- состав servo/PWM output packet;
- frame conventions;
- частоту обновления;
- порядок моторов;
- семантику throttle и PWM.

TASK-10 делает именно это согласование на уровне `.m`-кода,
без запуска внешнего SITL runtime.

## Чем TASK-10 отличается от PR #7

PR #7 с `atc_controller` остается отдельной research/integration branch
для controller-in-the-loop MIL boundary с внешним локальным контроллером.

TASK-10 не зависит от PR #7 и не переносит изменения из этой ветки.

Разница по смыслу такая:

- PR #7 исследует MIL bridge с отдельным локальным controller backend;
- TASK-10 готовит boundary именно под будущий `ArduPilot JSON SITL`;
- актуальный `main` после PR #9 и PR #10 используется как единственная
  база для TASK-10.

## Что реализовано в TASK-10

В пакете `src/+uav/+ardupilot/` добавлены:

- `default_json_config.m`
- `inspect_sitl_environment.m`
- `pack_json_fdm_packet.m`
- `unpack_servo_outputs.m`
- `pwm_to_motor_radps.m`
- `make_loopback_servo_packet.m`
- `validate_json_packet.m`

Также добавлены:

- `uav.sim.run_case_with_ardupilot_loopback`
- `scripts/inspect_ardupilot_sitl_env.m`
- `scripts/run_ardupilot_loopback_hover.m`
- `scripts/run_ardupilot_loopback_yaw_step.m`
- новые unit и smoke tests для adapter scaffold

## Frame conventions

Для TASK-10 используются следующие явные соглашения:

- earth frame: `NED`
- body frame: `FRD`
- quaternion: `q_nb`, scalar-first, body-to-NED
- body angular rates: `w_b_radps`
- velocity in Earth frame: `velocity_ned_mps`
- body velocity: `v_body_mps`

Важно:
в адаптере не смешиваются `NED` и `ENU`,
а body velocity не подменяется velocity в земной системе координат.

## Состав canonical FDM packet

`pack_json_fdm_packet` формирует MATLAB struct,
который фиксирует будущую boundary-схему.

Минимально в packet входят:

- `time_s`
- `position_ned_m`
- `velocity_ned_mps`
- `q_nb`
- `w_b_radps`
- `imu.accel_b_mps2`
- `imu.gyro_b_radps`
- `baro.alt_m`
- `baro.pressure_pa`
- `mag.field_b_uT`
- `gnss.pos_ned_m`
- `gnss.vel_ned_mps`

На этом шаге packet не сериализуется в JSON и не передается по UDP.

## Servo / PWM boundary

TASK-10 пока поддерживает только canonical input:

- `packet.pwm_us`

Функция `unpack_servo_outputs` нормализует этот packet,
а `pwm_to_motor_radps` делает явное преобразование:

1. PWM нормируется в диапазон `[0, 1]`
2. затем mapping переводит значение в диапазон
   `params.motor.omega_min_radps .. omega_max_radps`
3. saturation выполняется явно

Скрытых параметров в этом mapping нет.

## Что такое loopback smoke

`uav.sim.run_case_with_ardupilot_loopback` использует существующие:

- plant;
- sensor layer;
- estimator layer.

На каждом шаге он делает цепочку:

`plant -> sensors -> estimator -> ArduPilot-style packet -> fake servo/PWM -> motor rad/s -> plant`

Вместо реального ArduPilot SITL используется
`make_loopback_servo_packet`.

Поддерживаются два smoke режима:

- `hover`
- `yaw_step`

Эти режимы нужны не для валидации автопилота,
а для проверки boundary и численно прозрачного loopback path.

## Что НЕ утверждается на этом шаге

TASK-10 не заявляет:

- реальную ArduPilot интеграцию;
- устойчивый closed-loop hover с ArduPilot;
- корректную arming/mode handling логику;
- окончательное соответствие motor order;
- окончательное соответствие PWM semantics;
- финальную настройку параметров под реальный внешний автопилот.

## Локальная inventory-проверка среды

`inspect_sitl_environment` не падает,
если ArduPilot локально отсутствует.

Он проверяет только:

- наличие WSL, если это Windows-host;
- наличие Python;
- наличие `sim_vehicle.py` в `PATH`, если доступно;
- наличие локального ArduPilot checkout, если путь задан.

Именно эта inventory-проверка должна честно показать,
готова ли среда к следующему реальному SITL шагу.

## Следующий шаг для реального ArduPilot SITL

После TASK-10 следующим отдельным этапом должны стать:

1. подтверждение локальной готовности WSL / Python / ArduPilot checkout
2. запуск `sim_vehicle.py`
3. реальный UDP JSON bridge
4. alignment по motor order и packet semantics
5. alignment по parameter mapping
6. обработка arming / takeoff / mode handling
7. уже затем - первые реальные SITL smoke runs

Иными словами,
TASK-10 фиксирует интерфейс и scaffold,
но не подменяет собой будущую полноценную интеграцию.
