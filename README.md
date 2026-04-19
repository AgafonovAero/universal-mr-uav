# Universal MR UAV

`universal-mr-uav` — это kernel-first и code-centric репозиторий для
универсальной многороторной математической модели движения в
MATLAB/Simulink.

На текущем этапе source of truth остается текстовым:

- `.m`-код с физикой объекта управления, sensor layer и estimator layer;
- markdown-документация;
- raw logs локальных MATLAB/Simulink-прогонов;
- CSV/MAT/PNG артефакты воспроизводимых demo и verification-сценариев.

Simulink в проекте разрешен только как thin orchestration shell поверх
существующего `.m`-ядра. Ни `models/mil_top.slx`, ни `models/sil_top.slx`
не являются источником истины и поддерживаются только через
script-generated `.m`-workflow.

## Текущая стадия

Репозиторий находится на стадии `Stage-1.5+`.

На этой стадии уже готовы:

- code-centric ядро объекта управления;
- sensor layer в `.m`-коде;
- estimator layer в `.m`-коде;
- минимальный thin MIL shell;
- SIL-prep interface layer для следующего шага по внешним flight stack;
- estimator-driven closed-loop demo сценарии для takeoff и pitch-step.

## Roadmap по внешним flight stack

Приоритет следующей интеграции задан явно:

1. `ArduPilot SIL`
2. `PX4 SIL`
3. `PX4 HIL`

Важно: текущий этап не делает реальный runtime bridge по `MAVLink` или
`UDP` и не запускает настоящие ArduPilot/PX4 runtimes. Здесь фиксируется
архитектурная граница, интерфейсный слой и воспроизводимая база для
следующего engineering step.

## Что уже есть в репозитории

### Code-centric ядро

- математическая модель движения `6DOF` на кватернионах;
- простая модель ВМГ `T = kT * omega^2`, `Q = kQ * omega^2`;
- quad-X геометрия и code-centric суммирование сил и моментов;
- отдельные пакеты для `core`, `vmg`, `env`, `sim`.

### Sensor layer

В code-centric sensor layer уже реализованы:

- `IMU`;
- `barometer`;
- `magnetometer`;
- `GNSS`.

### Estimator layer

В estimator layer уже есть:

- complementary filter ориентации;
- gating accelerometer correction по consistency specific force;
- complementary filter высоты;
- явная diagnostic-информация для postprocessing и verification.

### Demo-level controllers и runner

Для estimator-driven verification-сценариев добавлены:

- `uav.ctrl.demo_takeoff_hold_controller`;
- `uav.ctrl.demo_pitch_hold_controller`;
- `uav.sim.run_case_closed_loop_with_estimator`.

### Thin Simulink shells

В репозитории поддерживаются два thin shell уровня orchestration:

- thin MIL shell через `uav.sl.Stage15MILSystem` и script-generated
  `models/mil_top.slx`;
- thin SIL-prep shell через `uav.sil.*`,
  `uav.sl.Stage15SILBridgeSystem`,
  `uav.sl.StubExternalFCSSystem` и script-generated
  `models/sil_top.slx`.

### Verification и артефакты

Также в репозитории уже есть:

- unit tests и integration tests;
- demo scripts для estimator-driven полета;
- raw logs локальных прогонов;
- PNG/MAT/CSV артефакты в `artifacts/`.

## Архитектурный принцип

Репозиторий остается `kernel-first` и `text-first`.

Это означает следующее:

- физика объекта управления остается в `src/+uav/+core`,
  `src/+uav/+vmg` и `src/+uav/+sim`;
- sensor layer остается в `src/+uav/+sensors`;
- estimator layer остается в `src/+uav/+est`;
- пакет `src/+uav/+sil` фиксирует канонический внешний интерфейс для
  будущих adapters;
- пакет `src/+uav/+sl` содержит только thin orchestration wrappers и bus
  definitions;
- `.slx` не должен содержать ручной block-diagram реализации physics,
  sensor layer или estimator layer.

## Канонические соглашения

В проекте используются следующие базовые соглашения:

- земная система координат в ядре: `NED`;
- связанная система координат: `X` вперед, `Y` вправо, `Z` вниз;
- внутренняя ориентация: quaternion `q_nb`;
- углы Эйлера допустимы только для display, debug и selected interfaces;
- единицы измерения: только `SI`;
- углы в коде: только `rad`.

## Быстрый старт и базовая локальная проверка

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

## Как посмотреть estimator-driven flight demos

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');

run('scripts/run_demo_takeoff_to_50m.m');
run('scripts/plot_demo_takeoff_to_50m.m');

run('scripts/run_demo_pitch_step_minus10deg.m');
run('scripts/plot_demo_pitch_step_minus10deg.m');
```

После этого появятся воспроизводимые flight-demo артефакты.

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

Plot scripts не переносят физику в `.slx`: они только читают сохраненные
MAT-артефакты и строят PNG.

Важно: demo controllers в TASK-09 больше не используют true-state
feedback. Они получают только estimator outputs, reference signals и
измеренные body rates IMU, что делает demos ближе к следующему шагу по
внешней системе автоматического управления.

## Локальные артефакты и raw logs

Для каждого завершенного engineering step в репозитории должны
оставаться:

- raw logs локальных прогонов в `artifacts/logs/`;
- markdown summary-файлы с ограничениями и допущениями;
- воспроизводимые PNG/CSV/MAT артефакты demo и verification-сценариев.

Это нужно для того, чтобы утверждения о локальных прогонах и верификации
опирались не на пересказ, а на текстовые и численные артефакты внутри
репозитория.

## Что именно добавили TASK-08 и TASK-09

### TASK-08

TASK-08 не интегрирует реальный flight stack. Он делает только
подготовительный SIL interface layer:

- канонический внешний actuator packet;
- канонический внешний sensor packet;
- прозрачный multi-rate scheduler для packet boundary;
- bridge between external actuator command and code-centric
  plant/sensors/estimator;
- временный `StubExternalFCSSystem` для smoke tests;
- отдельный `sil_top.slx`, не ломающий `mil_top.slx`.

### TASK-09

TASK-09 делает следующий шаг к честной closed-loop verification:

- улучшает attitude estimator в ускоренном наклоненном полете;
- переводит demo-сценарии на estimator-driven closed loop;
- добавляет diagnostic-поля для оценки consistency/gating;
- сохраняет physics и estimator logic в `.m`-коде, а не в `.slx`.

## Что важно не ломать в следующих задачах

При дальнейшем движении к внешним flight stack важно сохранять уже
зафиксированные ограничения:

- не переносить физику объекта управления в `.slx`;
- не переносить sensor layer и estimator layer в ручные block diagrams;
- не использовать `.slx` как source of truth;
- не подменять estimator-driven контуры true-state feedback'ом в demo и
  verification-сценариях.

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

## Документы

Ключевые документы по архитектуре и API:

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `docs/20_plant_api_ru.md`
- `docs/30_sensor_api_ru.md`
- `docs/40_estimator_api_ru.md`
- `docs/50_simulink_shell_ru.md`
- `docs/60_sil_interface_ru.md`
