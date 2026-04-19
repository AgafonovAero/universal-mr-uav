# Universal MR UAV

`universal-mr-uav` - это kernel-first и code-centric репозиторий для
универсальной многороторной математической модели движения в
MATLAB/Simulink.

Главный архитектурный принцип проекта простой:
source of truth должен оставаться в текстовом `.m`-коде и markdown-файлах,
а Simulink допускается только как тонкая orchestration-оболочка поверх
существующего ядра.

## Текущая стадия

Репозиторий находится на стадии `Stage-1.5+`.

На текущем этапе уже существуют:

- code-centric ядро математической модели движения;
- sensor layer в `.m`-коде;
- estimator layer в `.m`-коде;
- thin MIL shell;
- SIL-prep boundary для внешних flight stack;
- estimator-driven flight demos из TASK-09;
- ArduPilot JSON SITL adapter scaffold из TASK-10.

## Что считается source of truth

На текущем этапе источником истины считаются:

- `.m`-функции с физикой объекта управления;
- `.m`-реализация sensor layer;
- `.m`-реализация estimator layer;
- `.m`-реализация adapter и bridge layers;
- markdown-документация;
- raw logs локальных прогонов;
- воспроизводимые CSV, MAT и PNG артефакты.

`models/mil_top.slx` и `models/sil_top.slx` не являются source of truth.
Они поддерживаются только как thin shell поверх существующего
code-centric ядра и не должны становиться местом ручного переноса физики,
sensor layer или estimator layer.

## Приоритет roadmap

Текущий внешний integration roadmap зафиксирован так:

1. `ArduPilot SIL`
2. `PX4 SIL`
3. `PX4 HIL`

Это означает, что сначала фиксируется прозрачная граница интерфейса,
а уже затем поднимается реальный runtime bridge с внешним автопилотом.

## Что уже реализовано

### Code-centric ядро

В ядре уже есть:

- математическая модель движения `6DOF`;
- quaternion-based представление ориентации;
- простая модель ВМГ
  `T = kT * omega^2`, `Q = kQ * omega^2`;
- quad-X микшер;
- явное суммирование сил и моментов;
- пакетная структура `core`, `vmg`, `env`, `sim`.

### Sensor layer

В `src/+uav/+sensors` уже реализованы:

- `IMU`;
- `barometer`;
- `magnetometer`;
- `GNSS`.

### Estimator layer

В `src/+uav/+est` уже реализованы:

- complementary filter ориентации;
- accelerometer gating по specific-force consistency;
- complementary filter высоты;
- явные диагностические поля для verification и postprocessing.

### Demo-level closed loop

В TASK-09 добавлены:

- `uav.ctrl.demo_takeoff_hold_controller`;
- `uav.ctrl.demo_pitch_hold_controller`;
- `uav.sim.run_case_closed_loop_with_estimator`.

Эти demo-level controllers используют estimator outputs и не используют
true-state feedback.

### Thin Simulink shell

В репозитории поддерживаются два orchestration-уровня:

- thin MIL shell через `uav.sl.Stage15MILSystem` и
  script-generated `models/mil_top.slx`;
- thin SIL-prep shell через `uav.sil.*`,
  `uav.sl.Stage15SILBridgeSystem`,
  `uav.sl.StubExternalFCSSystem` и
  script-generated `models/sil_top.slx`.

### ArduPilot JSON SITL scaffold

В TASK-10 добавлен отдельный пакет `src/+uav/+ardupilot/`.

На этом шаге реализованы:

- явный config для будущего JSON SITL adapter;
- environment inspection без падения при отсутствии ArduPilot;
- pack/unpack functions для FDM packet и servo/PWM packet;
- PWM -> motor rad/s mapping без скрытых параметров;
- loopback smoke runner без реального ArduPilot SITL;
- scripts и tests для boundary verification.

Важно:
это именно scaffold и loopback smoke level,
а не полноценная интеграция с запущенным `sim_vehicle.py`.

## Чем TASK-10 отличается от PR #7

PR #7 с `atc_controller` остается отдельной research/integration branch
для controller-in-the-loop MIL boundary с внешним локальным контроллером.

TASK-10 не зависит от PR #7 и не переносит его изменения.

Разделение ролей такое:

- PR #7 проверяет отдельный MIL bridge для внешнего `atc_controller`;
- TASK-10 фиксирует будущую ArduPilot JSON SITL boundary на актуальном
  `main`;
- следующий реальный шаг по ArduPilot будет строиться поверх
  `uav.ardupilot.*`, а не поверх ветки PR #7.

## Канонические соглашения

В проекте используются следующие базовые соглашения:

- земная система координат в ядре: `NED`;
- связанная система координат:
  `X` вперед,
  `Y` вправо,
  `Z` вниз;
- внутренняя ориентация: quaternion `q_nb`;
- Euler angles допускаются только для display, debug и отдельных
  интерфейсов;
- единицы измерения: только `SI`;
- углы в коде: только `rad`.

## Быстрый локальный старт

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');

run('scripts/build_mil_top.m');
run('scripts/run_mil_top_hover.m');
run('scripts/run_mil_top_yaw_step.m');

run('scripts/build_sil_top.m');
run('scripts/run_sil_stub_hover.m');
run('scripts/run_sil_stub_yaw_step.m');

results = runtests('tests');
table(results)
```

Минимальный обязательный набор локальной проверки для инженерной задачи:

- `scripts/bootstrap_project.m`
- `runtests('tests')`
- task-specific demo scripts
- сохранение raw logs в `artifacts/logs/`
- обновление task summary-файла в `artifacts/reports/`

## Как посмотреть estimator-driven flight demos

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');

run('scripts/run_demo_takeoff_to_50m.m');
run('scripts/plot_demo_takeoff_to_50m.m');

run('scripts/run_demo_pitch_step_minus10deg.m');
run('scripts/plot_demo_pitch_step_minus10deg.m');
```

После этого появляются воспроизводимые flight-demo артефакты.

PNG-графики сохраняются в `artifacts/figures/`:

- `demo_takeoff_altitude.png`
- `demo_takeoff_altitude_error.png`
- `demo_takeoff_vertical_speed.png`
- `demo_takeoff_motor_cmd.png`
- `demo_pitch_angle.png`
- `demo_pitch_true_vs_estimated.png`
- `demo_pitch_estimation_error.png`
- `demo_pitch_altitude.png`
- `demo_pitch_motor_cmd.png`

Численные результаты сохраняются в `artifacts/reports/`:

- `demo_takeoff_to_50m.mat`
- `demo_takeoff_to_50m.csv`
- `demo_pitch_step_minus10deg.mat`
- `demo_pitch_step_minus10deg.csv`

## Как посмотреть TASK-10 ArduPilot loopback scaffold

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');

run('scripts/inspect_ardupilot_sitl_env.m');
run('scripts/run_ardupilot_loopback_hover.m');
run('scripts/run_ardupilot_loopback_yaw_step.m');
```

Эти скрипты не запускают реальный ArduPilot SITL.

Они проверяют только:

- локальную готовность среды;
- pack/unpack boundary;
- PWM semantics;
- loopback smoke path
  `plant -> sensors -> estimator -> ArduPilot-style packet -> fake PWM -> plant`.

## Что пока не реализовано для реального ArduPilot SITL

TASK-10 сознательно не делает следующие шаги:

- не поднимает реальный `sim_vehicle.py`;
- не вендорит внешний ArduPilot source tree в этот репозиторий;
- не реализует реальный UDP JSON bridge;
- не фиксирует финальное соответствие motor order и PWM semantics;
- не реализует arming, takeoff и mode handling уровня production;
- не заявляет полноценный полет с внешним ArduPilot SITL.

## Локальные артефакты и raw logs

Для каждого завершенного engineering step в репозитории должны оставаться:

- raw logs локальных прогонов в `artifacts/logs/`;
- markdown summary-файлы с ограничениями и допущениями;
- воспроизводимые PNG, CSV и MAT артефакты demo и verification-сценариев.

Это нужно для того, чтобы утверждения о локальной проверке опирались не на
пересказ, а на текстовые и численные артефакты внутри репозитория.

## Ключевые документы

Основные документы по архитектуре и API:

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `docs/20_plant_api_ru.md`
- `docs/30_sensor_api_ru.md`
- `docs/40_estimator_api_ru.md`
- `docs/50_simulink_shell_ru.md`
- `docs/60_sil_interface_ru.md`
- `docs/70_ardupilot_json_sitl_ru.md`

Task-level summaries и run logs лежат в:

- `artifacts/reports/`
- `artifacts/logs/`

## Что важно не ломать дальше

При дальнейшем движении к внешним flight stack важно сохранять уже
зафиксированные ограничения:

- не переносить физику объекта управления в `.slx`;
- не переносить sensor layer и estimator layer в ручные block diagrams;
- не использовать `.slx` как source of truth;
- не подменять estimator-driven контуры true-state feedback в demo и
  verification-сценариях;
- не превращать loopback smoke в ложное утверждение о готовом
  ArduPilot runtime bridge.

Именно эти ограничения позволяют держать репозиторий reviewable,
воспроизводимым и пригодным для дальнейшей внешней интеграции.
