# Universal MR UAV

`universal-mr-uav` - это kernel-first и code-centric репозиторий для
универсальной многороторной математической модели движения в
MATLAB/Simulink.

Главный архитектурный принцип проекта простой:
source of truth должен оставаться в текстовом `.m`-коде и markdown-файлах,
а Simulink допускается только как тонкая orchestration-оболочка.

## Текущая стадия

Репозиторий находится на стадии `Stage-1.5+`.

На этой стадии уже существуют:

- code-centric ядро математической модели движения;
- sensor layer в `.m`-коде;
- estimator layer в `.m`-коде;
- минимальный thin MIL shell;
- SIL-prep interface layer для следующего шага по внешним flight stack;
- estimator-driven closed-loop demo-сценарии для takeoff и pitch-step.

## Что является source of truth

На текущем этапе источником истины считаются:

- `.m`-функции с физикой объекта управления;
- `.m`-реализация sensor layer;
- `.m`-реализация estimator layer;
- markdown-документация;
- raw logs локальных прогонов;
- воспроизводимые CSV, MAT и PNG артефакты.

`models/mil_top.slx` и `models/sil_top.slx` не являются source of truth.
Они поддерживаются только как thin shell поверх существующего
code-centric ядра и не должны становиться местом ручного переноса физики.

## Приоритет roadmap

Следующие внешние интеграционные шаги зафиксированы так:

1. `ArduPilot SIL`
2. `PX4 SIL`
3. `PX4 HIL`

Это означает, что текущая стадия не пытается поднять реальный runtime bridge
через `MAVLink`, `UDP` или полноценный внешний автопилот.
Сейчас фиксируются границы интерфейса, архитектура и воспроизводимая база
для следующих инженерных шагов.

## Что уже есть в репозитории

### Code-centric ядро

В ядре уже реализованы:

- математическая модель движения `6DOF`;
- quaternion-based представление ориентации;
- простая модель ВМГ
  `T = kT * omega^2`, `Q = kQ * omega^2`;
- quad-X геометрия;
- code-centric суммирование сил и моментов;
- пакетная структура `core`, `vmg`, `env`, `sim`.

### Sensor layer

В sensor layer уже присутствуют модели:

- `IMU`;
- `barometer`;
- `magnetometer`;
- `GNSS`.

### Estimator layer

В estimator layer уже присутствуют:

- complementary filter ориентации;
- gating accelerometer correction по consistency specific force;
- complementary filter высоты;
- явные диагностические поля для verification и postprocessing.

### Demo-level controllers и closed-loop runner

Для estimator-driven verification-сценариев добавлены:

- `uav.ctrl.demo_takeoff_hold_controller`;
- `uav.ctrl.demo_pitch_hold_controller`;
- `uav.sim.run_case_closed_loop_with_estimator`.

### Thin Simulink shell

В репозитории поддерживаются два orchestration-уровня:

- thin MIL shell через `uav.sl.Stage15MILSystem` и
  script-generated `models/mil_top.slx`;
- thin SIL-prep shell через `uav.sil.*`,
  `uav.sl.Stage15SILBridgeSystem`,
  `uav.sl.StubExternalFCSSystem` и
  script-generated `models/sil_top.slx`.

### Verification и артефакты

Также в репозитории уже есть:

- unit tests и integration tests;
- demo scripts для estimator-driven полета;
- raw logs локальных MATLAB-прогонов;
- PNG, MAT и CSV артефакты в `artifacts/`.

## Архитектурные правила

Репозиторий остается `kernel-first` и `text-first`.

Практически это означает следующее:

- физика объекта управления живет в `src/+uav/+core`,
  `src/+uav/+vmg` и `src/+uav/+sim`;
- sensor layer остается в `src/+uav/+sensors`;
- estimator layer остается в `src/+uav/+est`;
- пакет `src/+uav/+sil` фиксирует канонический внешний интерфейс;
- пакет `src/+uav/+sl` содержит только thin wrappers и bus definitions;
- `.slx` не должны становиться местом ручной block-diagram реализации
  physics, sensor layer или estimator layer.

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

Оба demo работают через estimator-driven code-centric runner
`uav.sim.run_case_closed_loop_with_estimator` по цепочке:

`объект управления -> sensor layer -> estimator layer -> demo controller -> ВМГ`

Plot scripts не переносят физику в `.slx`.
Они только читают уже сохраненные MAT-артефакты и строят PNG-графики.

Важно:
demo controllers в TASK-09 больше не используют true-state feedback.
Они получают только estimator outputs, reference signals и измеренные
body rates IMU.

## Локальные артефакты и raw logs

Для каждого завершенного engineering step в репозитории должны оставаться:

- raw logs локальных прогонов в `artifacts/logs/`;
- markdown summary-файлы с ограничениями и допущениями;
- воспроизводимые PNG, CSV и MAT артефакты demo и verification-сценариев.

Это нужно для того, чтобы утверждения о локальной проверке опирались
не на пересказ, а на текстовые и численные артефакты внутри репозитория.

## Что сделали TASK-08 и TASK-09

### TASK-08

TASK-08 не интегрирует реальный flight stack.
Он делает только подготовительный SIL interface layer:

- канонический внешний actuator packet;
- канонический внешний sensor packet;
- прозрачный multi-rate scheduler для packet boundary;
- bridge between external actuator command and code-centric
  plant, sensors и estimator;
- временный `StubExternalFCSSystem` для smoke tests;
- отдельный `sil_top.slx`, не ломающий `mil_top.slx`.

### TASK-09

TASK-09 делает следующий шаг к честной closed-loop verification:

- улучшает attitude estimator в ускоренном наклоненном полете;
- переводит demo-сценарии на estimator-driven closed loop;
- добавляет диагностические поля для consistency и gating;
- сохраняет physics и estimator logic в `.m`-коде, а не в `.slx`.

## Что важно не ломать дальше

При дальнейшем движении к внешним flight stack важно сохранять уже
зафиксированные ограничения:

- не переносить физику объекта управления в `.slx`;
- не переносить sensor layer и estimator layer в ручные block diagrams;
- не использовать `.slx` как source of truth;
- не подменять estimator-driven контуры true-state feedback в demo и
  verification-сценариях.

Именно эти ограничения позволяют держать репозиторий reviewable,
воспроизводимым и пригодным для дальнейшей внешней интеграции.

## Структура репозитория

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
src/+uav/+sil/
src/+uav/+sl/
models/
tests/
artifacts/logs/
artifacts/figures/
artifacts/reports/
```

## Ключевые документы

Основные документы по архитектуре и API:

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `docs/20_plant_api_ru.md`
- `docs/30_sensor_api_ru.md`
- `docs/40_estimator_api_ru.md`
- `docs/50_simulink_shell_ru.md`
- `docs/60_sil_interface_ru.md`
