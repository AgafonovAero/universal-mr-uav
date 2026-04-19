# ATC MIL Bridge Stage-1.5+

## Назначение шага

Этот шаг добавляет в `universal-mr-uav` отдельный bridge для внешнего
локального репозитория `atc_controller`.

Цель:

- подключить внешний controller layer к уже существующему thin MIL shell;
- не переносить физику объекта управления в `.slx`;
- не дублировать sensor layer и estimator layer;
- сохранить source of truth в `.m`-коде и текстовых файлах.

Это именно промежуточный controller-in-the-loop MIL шаг перед полноценным
stack-specific SIL.

## Текущий статус после актуализации main

После merge PR `#9` и PR `#10` эта ветка была повторно проверена на
совместимость с актуальным `main`.

Текущий вывод такой:

- ATC bridge по-прежнему совместим с текущим code-centric ядром;
- boundary по-прежнему использует существующие plant, sensors и estimator,
  а не переносит их в `.slx`;
- текущие `run_mil_atc_hover` и `run_mil_atc_yaw_step`
  остаются smoke-level boundary verification;
- эти demo не должны трактоваться как подтверждение настроенного
  и устойчивого closed-loop hover внешнего контроллера;
- полноценная настройка `atc_controller` под baseline airframe
  `universal-mr-uav` остаётся отдельным следующим этапом.

## Чем этот шаг отличается от базового thin MIL shell

Базовый thin MIL shell (`Stage15MILSystem`, `mil_top.slx`) ожидает уже готовую
команду на ВМГ извне.

Новый шаг добавляет:

- вызов внешнего контроллера внутри MATLAB System wrapper;
- явный adapter layer между внутренним Stage-1.5+ ядром и интерфейсом
  `atc_controller`;
- отдельную minimal script-generated модель `mil_top_atc.slx`.

При этом не меняется архитектурный принцип:

- математическая модель движения остается в `uav.sim.*` и `uav.core.*`;
- sensor layer остается в `uav.sensors.*`;
- estimator layer остается в `uav.est.*`;
- Simulink используется только как thin orchestration shell.

## Введенные интерфейсы

### Пакет `uav.atc`

Добавлен пакет:

```text
src/+uav/+atc/
```

Он фиксирует минимальный adapter layer.

#### `uav.atc.default_atc_bridge_config`

Назначение:

- собрать минимальный конфиг bridge;
- зафиксировать путь до внешнего репозитория;
- задать smoke-сценарий `hover` или `yaw_step`;
- явно хранить требования по quad-X и 4 моторам.

#### `uav.atc.ensure_atc_controller_on_path`

Назначение:

- вызвать `setup_paths` внешнего репозитория;
- проверить наличие прямых MATLAB entrypoint'ов.

#### `uav.atc.make_controller_context`

Назначение:

- загрузить `ATC_Params_default`;
- выполнить минимальную согласовку параметров с
  `universal-mr-uav`;
- подготовить mixer context и input template.

#### `uav.atc.pack_sensor_packet_for_atc`

Назначение:

- упаковать данные существующих `uav.sensors.*` и `uav.est.*` в fixed-field
  input struct для `FSW_Simulink_wrapper_step`.

Выбранный mapping:

- `gyro_rads` <- `sensors.imu.gyro_b_radps`
- `q_bn` <- `estimator.q_nb`
- `pos_ned(1:2)` <- `sensors.gnss.pos_ned_m(1:2)`
- `vel_ned(1:2)` <- `sensors.gnss.vel_ned_mps(1:2)`
- `pos_ned(3)` <- `-estimator.alt_m`
- `vel_ned(3)` <- `-estimator.vz_mps`

#### `uav.atc.unpack_atc_actuation`

Назначение:

- перевести `out.motor_cmd` внешнего контроллера в канонический actuator
  packet `universal-mr-uav`.

Используемая цепочка:

1. `motor_cmd` внешнего контроллера
2. `AP_MotorsMatrix_actuator_to_thrust`
3. `normalized thrust -> normalized omega`
4. `uav.sil.actuator_cmd_to_motor_radps`

## `Stage15ATCMILSystem`

Новый wrapper:

```text
src/+uav/+sl/Stage15ATCMILSystem.m
```

Он:

- хранит внутреннее состояние объекта управления;
- вызывает:
  - `uav.sim.plant_step_struct`
  - `uav.sensors.sensors_step`
  - `uav.est.estimator_step`
  - `uav.atc.*`
- вызывает внешний `FSW_Simulink_wrapper_step` через direct MATLAB call.

Выходы wrapper:

- `truth_out`
- `sensors_out`
- `estimator_out`
- `atc_cmd_out`
- `diag_out`

### Что принципиально не делается внутри wrapper

Не делается:

- ручная block-diagram реализация математической модели движения;
- перенос sensor layer в Simulink;
- перенос estimator layer в Simulink;
- построение нового MIL-контура с нуля.

## Script-generated top model

Добавлен builder:

```matlab
run('scripts/build_mil_top_atc.m');
```

Он создает:

```text
models/mil_top_atc.slx
```

В модели остается только:

- один MATLAB System block на базе `Stage15ATCMILSystem`;
- logging основных выходов.

Это сохраняет `.slx` thin-shell артефактом, а не source of truth.

## Что уже работает

На этом шаге работает:

- прямой MATLAB-вызов `atc_controller`;
- quad-X и 4 мотора;
- hover smoke demo;
- yaw-step smoke demo;
- script-generated `mil_top_atc.slx`;
- bridge от существующего Stage-1.5+ ядра до внешнего controller layer.

Важно:

- текущие demo-прогоны являются именно smoke-level boundary verification;
- внешний контроллер на этом шаге еще не отстроен как устойчивая система
  автоматического управления для baseline airframe `universal-mr-uav`.
- их назначение - подтвердить работоспособность adapter layer
  и controller-in-the-loop MIL boundary, а не заявить готовый closed-loop
  hover.

## Что пока остается ограничением

Ограничения шага:

- нет реального SIL runtime boundary;
- нет UDP/MAVLink/IPC transport;
- нет HIL;
- пока поддерживается только quad-X / 4 motors;
- используется minimal manual command profile, а не полный mission/autonomy
  pipeline;
- bridge честно рассчитан на direct MATLAB call, а не на защищенный бинарный
  black-box runtime.
- базовые hover/yaw-step demo пока показывают работоспособность boundary,
  но не подтверждают полноценную верификацию замкнутого контура для данной
  математической модели движения.

## Почему это отдельный промежуточный этап перед полноценным SIL

Полноценный SIL требует уже других инженерных решений:

- стабильного runtime contract;
- четкой упаковки sensor packet'ов и actuation packet'ов для конкретного
  flight stack;
- orchestration отдельного процесса/модуля контроллера;
- более жесткой верификации временных соглашений.

Если пытаться делать это сразу, растет риск:

- смешать архитектуру ядра и внешнего стека;
- затащить физику объекта управления в `.slx`;
- потерять прозрачность source of truth.

Поэтому текущий шаг осознанно фиксирует именно boundary:

- существующее `.m`-ядро остается неизменным по смыслу;
- внешний контроллер подключается через явный adapter layer;
- следующий шаг можно делать уже как более честный MIL/SIL upgrade, а не
  как архитектурный рефакторинг вслепую.
