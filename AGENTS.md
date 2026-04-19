# Project instructions

Project: Universal multirotor UAV model for MATLAB/Simulink

## Current stage

Stage-1.5+ with a minimal Simulink MIL shell and TASK-08 SIL-prep
interface layer.

Build and extend a repository with primacy of the computational core.

The subsystem of sensors and the state-estimation algorithm already exist
in `.m` code.

Current priority for external complex-of-flight-control integration:

1. ArduPilot SIL
2. PX4 SIL
3. PX4 HIL

Simulink is allowed only as a minimal orchestration shell over the
existing computational core.

Do NOT build GUI.

Do NOT build MLAPP apps in this task.

Do NOT create binary-only artifacts as the primary model description.

## Main engineering idea

The primary model description must remain MATLAB code and text files.

Simulink is allowed only as a minimal orchestration shell generated and
maintained through `.m` scripts. Simulink must never become the primary
source of physical logic.

## Terminology and documentation style

All documentation, reports, pull-request descriptions, review notes, task
results, and final answers for this project must be written in Russian
technical language.

Use terminology consistent with:

- ГОСТ Р 57258-2016
- ГОСТ 20058-80
- ГОСТ 23281-78
- ГОСТ 24999-81

In Russian text do not use colloquial English words for engineering
concepts if a Russian technical equivalent exists.

The following are allowed without translation:

- product names: MATLAB, Simulink, ArduPilot, PX4
- file names, function names, package names, branch names
- command lines and tool syntax
- standard symbols of physical quantities and units

Use the following Russian replacements in documentation and explanatory
text:

- `source of truth` -> исходное достоверное описание модели
- `code-centric` -> программно-реализованное расчетное ядро
- `kernel-first` -> первичность расчетного ядра
- `thin shell` -> тонкая имитационная оболочка
- `scaffold` -> заготовка средства сопряжения
- `loopback` -> проверочный замкнутый прогон
- `smoke` / `smoke test` -> первичная проверка работоспособности
- `boundary` -> граница сопряжения
- `adapter` -> модуль сопряжения
- `runner` -> исполнитель сценария моделирования
- `flight stack` -> внешний комплекс управления полетом
- `raw logs` -> исходные журналы прогонов
- `summary` -> отчет
- `roadmap` -> план-график работ
- `packet` -> пакет данных
- `mapping` -> преобразование
- `estimator` -> алгоритм оценивания состояния
- `sensor layer` -> подсистема датчиков
- `plant` -> объект управления
- `true state` -> истинное состояние объекта
- `hover` -> зависание
- `yaw-step` -> ступенчатое воздействие по рысканию
- `pitch-step` -> ступенчатое воздействие по тангажу
- `takeoff` -> взлет
- `debug` -> отладка
- `display` -> отображение
- `backend` -> исполняющая часть сопряженного комплекса
- `runtime bridge` -> средство обмена данными в ходе моделирования
- `production-ready` -> пригодный к штатной эксплуатации

If an external protocol or software complex must be mentioned for the first
time, first describe its Russian technical role, then give its exact proper
name.

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
- Euler angles are allowed only for display, debugging, and selected
  interfaces

## Units

Use SI only.

Angles in code must be radians.

If degrees are used anywhere, conversion must be explicit.

## Architectural rules

1. Keep the implementation text-first.
2. No `.mlapp`, `.sldd`, `.prj` creation in this task. A minimal `.slx`
   is allowed only when it is script-generated or script-maintained and
   stays a thin shell over the `.m` core.
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
10. If MATLAB execution is not available, explicitly say so and provide exact
    local commands to run.

## Modeling rules for this task

- Baseline plant: rigid-body 6DOF
- Attitude: quaternion-based
- Propulsion model: simple rotor model `T = kT * omega^2`,
  `Q = kQ * omega^2`
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
  flight stack boundary and the existing code-centric core
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

- Для каждой инженерной задачи работать в отдельной ветке вида
  `task/NN-short-name`.
- Основной режим работы Codex для этого репозитория: `Worktree`.
- После завершения каждой задачи обязательно:
  1. запустить `scripts/bootstrap_project.m`
  2. запустить `runtests('tests')`
  3. запустить специализированные демонстрационные сценарии задачи
  4. сохранить исходные журналы прогонов в `artifacts/logs/`
  5. обновить `artifacts/reports/task_NN_summary_ru.md`
  6. сделать commit
  7. выполнить push ветки
  8. открыть или обновить pull request
- Никогда не утверждать, что MATLAB-прогоны выполнены успешно, если
  исходные журналы прогонов не сохранены в репозитории.
- Все отчеты по задачам, пояснительные записки, сообщения фиксации,
  описания запросов на слияние и итоговые ответы писать на русском языке.

## Review guidelines

- Считать ошибками уровня P1:
  - ошибки знаков и систем координат
  - некорректную геометрию микширования
  - скрытые параметры вне preset/config файлов
  - отсутствие исходных журналов прогонов после заявленных запусков
  - перенос физической логики в бинарные артефакты вместо `.m`-кода
  - перенос физики объекта, подсистемы датчиков или алгоритма оценивания
    состояния внутрь `.slx` как ручной block-diagram реализации
- Требовать минимальный scope изменений без лишнего функционала.
