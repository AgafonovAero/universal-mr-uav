# Universal MR UAV

Минимальное code-centric ядро `Stage-1.5+` для универсального многороторного БПЛА в MATLAB/Simulink.

Источник истины в репозитории остается текстовым:

- `.m`-код
- документы
- raw logs локальных MATLAB/Simulink-прогонов

Simulink на текущем этапе разрешен только как thin orchestration shell поверх существующего `.m`-ядра. Ни `mil_top.slx`, ни `sil_top.slx` не являются source of truth и поддерживаются только через `.m`-скрипты.

## Текущая стадия и roadmap

- стадия: `Stage-1.5+`
- готово code-centric ядро объекта управления
- sensor layer и estimator layer уже реализованы в `.m`-коде
- есть минимальный thin MIL shell
- TASK-08 добавляет SIL-prep interface layer для будущего подключения внешних flight stack'ов

Приоритет roadmap для внешних стеков:

1. ArduPilot SIL
2. PX4 SIL
3. PX4 HIL

Важно: на шаге TASK-08 не выполняется реальная интеграция ArduPilot/PX4 и не строится runtime bridge по MAVLink/UDP. Фиксируется только граница интерфейса и минимальный SIL-prep shell.

## Что уже есть в репозитории

- математическая модель движения `6DOF` на кватернионах
- простая модель ВМГ `T = kT * omega^2`, `Q = kQ * omega^2`
- quad-X геометрия и code-centric суммирование сил и моментов
- sensor layer:
  - `IMU`
  - `barometer`
  - `magnetometer`
  - `GNSS`
- estimator layer:
  - complementary filter ориентации с gating по consistency specific force
  - complementary filter высоты
- estimator-driven demo controllers:
  - `uav.ctrl.demo_takeoff_hold_controller`
  - `uav.ctrl.demo_pitch_hold_controller`
- estimator-driven closed-loop runner:
  - `uav.sim.run_case_closed_loop_with_estimator`
- thin MIL shell:
  - `uav.sl.Stage15MILSystem`
  - script-generated `models/mil_top.slx`
- thin SIL-prep shell:
  - `uav.sil.*`
  - `uav.sl.Stage15SILBridgeSystem`
  - `uav.sl.StubExternalFCSSystem`
  - script-generated `models/sil_top.slx`
- unit tests, demo scripts и raw logs локальных прогонов

## Архитектурный принцип

Репозиторий остается kernel-first и text-first:

- физика объекта управления остается в `src/+uav/+core`, `src/+uav/+vmg`, `src/+uav/+sim`
- sensor layer остается в `src/+uav/+sensors`
- estimator layer остается в `src/+uav/+est`
- пакет `src/+uav/+sil` фиксирует канонический внешний интерфейс для будущих ArduPilot/PX4 adapters
- пакет `src/+uav/+sl` содержит только thin orchestration wrappers и bus definitions
- `.slx` не должен содержать ручной block-diagram реализации physics, sensor layer или estimator layer

## Канонические соглашения

- земная система координат в ядре: `NED`
- связанная система координат: `X` вперед, `Y` вправо, `Z` вниз
- внутренняя ориентация: quaternion `q_nb`
- углы Эйлера допустимы только для display/debug/interfaces
- единицы: только `SI`
- углы в коде: только `rad`

## Быстрый старт

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

## Как посмотреть, что модель действительно летает

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');

run('scripts/run_demo_takeoff_to_50m.m');
run('scripts/plot_demo_takeoff_to_50m.m');

run('scripts/run_demo_pitch_step_minus10deg.m');
run('scripts/plot_demo_pitch_step_minus10deg.m');
```

После этого в репозитории появятся воспроизводимые flight-demo артефакты:

- PNG-графики в `artifacts/figures/`:
  - `demo_takeoff_altitude.png`
  - `demo_takeoff_altitude_error.png`
  - `demo_takeoff_vertical_speed.png`
  - `demo_takeoff_motor_cmd.png`
  - `demo_pitch_angle.png`
  - `demo_pitch_true_vs_estimated.png`
  - `demo_pitch_estimation_error.png`
  - `demo_pitch_altitude.png`
  - `demo_pitch_motor_cmd.png`
- численные результаты в `artifacts/reports/`:
  - `demo_takeoff_to_50m.mat`
  - `demo_takeoff_to_50m.csv`
  - `demo_pitch_step_minus10deg.mat`
  - `demo_pitch_step_minus10deg.csv`

Эти сценарии не переносят физику в `.slx`: оба demo работают поверх
estimator-driven code-centric runner
`uav.sim.run_case_closed_loop_with_estimator` по цепочке
`объект управления -> sensor layer -> estimator layer -> demo controller -> ВМГ`,
а plot scripts только читают сохраненные MAT-артефакты и строят PNG.

Важно: demo controllers в TASK-09 больше не используют true-state feedback.
Они получают только estimator outputs, reference signals и измеренные body
rates IMU, что делает demos ближе к будущей внешней системе
автоматического управления.

## TASK-08: что именно добавлено

TASK-08 не интегрирует реальный flight stack. Он делает только подготовительный SIL interface layer:

- канонический внешний actuator packet
- канонический внешний sensor packet
- прозрачный multi-rate scheduler для packet boundary
- bridge between external actuator command and code-centric plant/sensors/estimator
- временный `StubExternalFCSSystem` для smoke tests
- отдельный `sil_top.slx`, не ломающий `mil_top.slx`

Это нужно для того, чтобы следующие задачи были уже стек-специфичными, а не архитектурно-размытыми.

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
src/+uav/+sil/
src/+uav/+sl/
models/
tests/
artifacts/logs/
artifacts/figures/
artifacts/reports/
```

## Документы

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `docs/20_plant_api_ru.md`
- `docs/30_sensor_api_ru.md`
- `docs/40_estimator_api_ru.md`
- `docs/50_simulink_shell_ru.md`
- `docs/60_sil_interface_ru.md`
