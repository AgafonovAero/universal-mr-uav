# Project instructions

Project: Universal multirotor UAV model for MATLAB/Simulink

## Current stage

Stage-1.5+ with minimal thin Simulink MIL shell and TASK-08 SIL-prep
interface layer.

Build and extend a code-centric kernel-first repository.

Sensor layer and estimator layer already exist in `.m` code.

Current roadmap priority for external stack integration:

1. ArduPilot SIL
2. PX4 SIL
3. PX4 HIL

Simulink is allowed only as a thin orchestration shell over the existing
code-centric kernel.

Do NOT build GUI.

Do NOT build MLAPP apps in this task.

Do NOT create binary-only artifacts as the main source of truth.

## Main engineering idea

The source of truth must remain MATLAB code and text files.

Simulink is allowed only as a thin orchestration shell that is generated
and maintained through `.m` scripts and never becomes the source of truth.

## Domain language

Use Russian flight-dynamics terminology in docs:

- объект управления
- математическая модель движения
- связанная система координат
- земная система координат
- ВМГ
- система автоматического управления
- контур угловой стабилизации
- контур стабилизации высоты
- траекторное управление
- верификация
- валидация
- идентификация
- летные испытания

Use concise English identifiers in code.

## Coordinate conventions

- Earth frame in core: NED
- Body frame: X forward, Y right, Z down
- Internal attitude representation: quaternion
- Euler angles are allowed only for display, debugging, and selected interfaces

## Units

Use SI only.

Angles in code must be radians.

If degrees are used anywhere, conversion must be explicit.

## Architectural rules

1. Keep the implementation text-first.
2. No `.mlapp`, `.sldd`, `.prj` creation in this task. A minimal `.slx`
   is allowed only when it is script-generated or script-maintained and
   stays a thin shell over the `.m` kernel.
3. Use MATLAB package folders under `/src/+uav/`.
4. Separate code into:
   - `core`
   - `vmg`
   - `env`
   - `ctrl`
   - `sensors`
   - `est`
   - `sim`
   - `sil`
   - `sl`
5. Every nontrivial function must include:
   - H1 line
   - description
   - inputs
   - outputs
   - units
   - assumptions
6. No hardcoded aircraft values inside generic functions.
7. Put example parameters into dedicated preset files.
8. Keep the first implementation minimal and runnable.
9. Do not fake test execution results.
10. If MATLAB execution is not available, explicitly say so and provide exact local commands to run.

## Modeling rules for this task

- Baseline plant: rigid-body 6DOF
- Attitude: quaternion-based
- Propulsion model: simple rotor model `T = kT * omega^2`, `Q = kQ * omega^2`
- Mixer: quad-X only for now
- Control: minimal body-rate PID scaffold only
- Sensor layer: code-centric and explicit
- Estimator layer: code-centric and explicit
- Environment: gravity only for now
- One basic hover demo
- Two basic unit tests minimum
- MIL thin shell is allowed only as orchestration around existing `.m`
  APIs for plant, sensors, and estimator
- TASK-08 prepares only a SIL-prep interface layer between an external
  flight stack boundary and the existing code-centric kernel
- TASK-08 does NOT implement a real MAVLink, UDP, ArduPilot, or PX4
  runtime bridge
- `models/mil_top.slx` and existing TASK-07 scenarios must remain
  available and must not be broken by TASK-08 changes

## Done criteria

A task is done only if:

- requested files are created
- repository structure is coherent
- code is readable
- tests are provided
- local verification commands are provided
- assumptions and limitations are written down

## Git workflow

- Для каждой инженерной задачи работать в отдельной ветке вида `task/NN-short-name`.
- Основной режим работы Codex для этого репозитория: `Worktree`.
- После завершения каждой задачи обязательно:
  1. запустить `scripts/bootstrap_project.m`
  2. запустить `runtests('tests')`
  3. запустить task-specific demo scripts
  4. сохранить сырые логи в `artifacts/logs/`
  5. обновить `artifacts/reports/task_NN_summary_ru.md`
  6. сделать commit
  7. выполнить push ветки
  8. открыть или обновить pull request
- Никогда не утверждать, что MATLAB-прогоны выполнены успешно, если сырые логи не сохранены в репозитории.
- Все task reports, summary files, review notes, commit messages и PR descriptions писать на русском языке.

## Review guidelines

- Считать ошибками уровня P1:
  - ошибки знаков и систем координат,
  - некорректную геометрию микширования,
  - скрытые параметры вне preset/config файлов,
  - отсутствие raw logs после заявленных прогонов,
  - перенос физической логики в бинарные артефакты вместо `.m` кода,
  - перенос физики объекта, sensor layer или estimator layer внутрь
    `.slx` как ручной block-diagram реализации.
- Требовать минимальный scope изменений без лишнего функционала.
