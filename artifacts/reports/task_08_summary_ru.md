# TASK-08: SIL-prep interface layer

## Что сделано

В рамках TASK-08 репозиторий подготовлен к приоритетному SIL-подключению внешних flight stack'ов без разрушения code-centric архитектуры.

Сделано:

- обновлены `AGENTS.md`, `README.md`, `docs/50_simulink_shell_ru.md`
- добавлен `docs/60_sil_interface_ru.md`
- добавлен пакет `src/+uav/+sil/`:
  - `actuator_cmd_to_motor_radps`
  - `hover_trim_from_params`
  - `make_sensor_packet`
  - `update_rate_scheduler`
- добавлены thin Simulink wrappers:
  - `uav.sl.Stage15SILBridgeSystem`
  - `uav.sl.StubExternalFCSSystem`
  - `uav.sl.make_sil_bus_defs`
- добавлен script-generated builder `scripts/build_sil_top.m`
- добавлены demo scripts:
  - `scripts/run_sil_stub_hover.m`
  - `scripts/run_sil_stub_yaw_step.m`
- добавлены тесты TASK-08
- создан `models/sil_top.slx` как отдельный thin SIL-prep shell

При этом:

- `models/mil_top.slx` сохранен
- существующие TASK-07 MIL-сценарии не удалялись и не заменялись
- plant/sensor/estimator logic не переносились внутрь block diagram

## Что реально запускалось локально

Локально выполнены и сохранены в raw logs:

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/build_sil_top.m`
4. `scripts/run_sil_stub_hover.m`
5. `scripts/run_sil_stub_yaw_step.m`

Raw logs:

- `artifacts/logs/task_08_bootstrap.txt`
- `artifacts/logs/task_08_runtests.txt`
- `artifacts/logs/task_08_build_sil_top.txt`
- `artifacts/logs/task_08_run_sil_stub_hover.txt`
- `artifacts/logs/task_08_run_sil_stub_yaw_step.txt`

## Результаты тестов

По `artifacts/logs/task_08_runtests.txt`:

- полный прогон `runtests('tests')` завершился успешно
- пройдено `39` тестов
- `failed = 0`
- `incomplete = 0`

Дополнительно TASK-08 проверяет:

- преобразование `motor_norm_01 -> motor_cmd_radps`
- saturation по нормированному входу и моторным пределам
- создание SIL bus definitions
- программную сборку `models/sil_top.slx`
- выполнение `run_sil_stub_hover`
- согласованность `Stage15SILBridgeSystem` с существующим code-centric API

## Результаты demo-сценариев

### `run_sil_stub_hover`

По `artifacts/logs/task_08_run_sil_stub_hover.txt`:

- final true position NED: `[0.000000 0.000000 0.791605]`
- final true altitude: `-0.791605 m`
- final estimated altitude: `-0.785766 m`
- final true quat norm: `1.000000000000`
- final estimated quat norm: `1.000000000000`
- final hover actuator command: `[0.631033 0.631033 0.631033 0.631033]`

### `run_sil_stub_yaw_step`

По `artifacts/logs/task_08_run_sil_stub_yaw_step.txt`:

- final yaw estimate: `1.469076 rad`
- final yaw rate: `2.195144 rad/s`
- final altitude: `-0.772737 m`
- final estimated altitude: `-0.768108 m`
- final true quat norm: `1.000000000000`
- final estimated quat norm: `1.000000000000`

## Почему этот шаг считается подготовкой к ArduPilot/PX4 SIL

TASK-08 зафиксировал именно ту границу, которая нужна для следующего стек-специфичного шага:

- внешний actuator interface
- внешний sensor packet
- packet-level multi-rate scheduler
- отдельный thin Simulink shell `sil_top.slx`
- bridge между внешним FCS boundary и существующим `.m`-ядром

То есть теперь есть не абстрактная идея SIL, а минимальная воспроизводимая boundary-конструкция, на которую можно уже навешивать реальный adapter конкретного flight stack'а.

## Почему реальная интеграция стеков вынесена в следующие задачи

TASK-08 сознательно не включает реальную интеграцию ArduPilot/PX4, потому что для нее нужны отдельные инженерные решения по:

- transport/runtime contract
- стек-специфичному mapping'у actuator outputs
- стек-специфичному packaging sensor data
- orchestration и запуску конкретного flight stack runtime

Если делать это раньше фиксации общей boundary-архитектуры, возрастает риск смешать стек-специфику с ядром и превратить `.slx` в место, где живет физическая логика. TASK-08 именно этого не допускает.

## Какие ограничения остаются

На конец TASK-08 остаются ограничения:

- нет реального ArduPilot runtime bridge
- нет реального PX4 runtime bridge
- нет MAVLink/UDP transport layer
- нет HIL
- внешний controller пока представлен только временным `StubExternalFCSSystem`
- `sil_top.slx` остается минимальным orchestration shell, а не полноценной средой интеграции flight stack runtime

## Почему следующим шагом логично делать именно ArduPilot SIL

После TASK-08 уже готовы:

- канонический actuator packet
- канонический sensor packet
- deterministic scheduler boundary
- отдельный `sil_top.slx`
- smoke-tested bridge между внешним FCS и code-centric kernel

Поэтому следующий логичный шаг не в дальнейшем абстрактном рефакторинге shell, а в первом реальном stack-specific adapter. По текущему roadmap таким шагом должен быть именно `ArduPilot SIL`, затем `PX4 SIL`, и только после этого `PX4 HIL`.
