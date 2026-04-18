# TASK-07 Summary

## Что сделано

В рамках `TASK-07` поверх существующего code-centric ядра `Stage-1.5+` добавлен минимальный thin Simulink MIL shell без переноса физики объекта управления, sensor layer или estimator layer внутрь `.slx`.

Сделано:

- обновлены `AGENTS.md` и `README.md`:
  - зафиксирована стадия `Stage-1.5+`
  - явно указано, что `sensor layer` и `estimator layer` уже существуют
  - разрешён только минимальный script-generated `.slx` как thin orchestration shell
- добавлен пакет `src/+uav/+sl/`:
  - `Stage15MILSystem.m`
  - `make_bus_defs.m`
  - `make_demo_command_profile.m`
- добавлен script-generated thin shell:
  - `scripts/build_mil_top.m`
  - `models/mil_top.slx`
- добавлены demo-сценарии:
  - `scripts/run_mil_top_hover.m`
  - `scripts/run_mil_top_yaw_step.m`
- добавлены Simulink-oriented tests:
  - `tests/test_make_bus_defs.m`
  - `tests/test_build_mil_top.m`
  - `tests/test_run_mil_top_hover.m`
- добавлена документация:
  - `docs/50_simulink_shell_ru.md`
- удалён лишний исторический файл `TASK_05_RU.md`, чтобы ветка осталась минимальной по scope

## Что реально запускалось локально

Локально на этой машине с установленным MATLAB/Simulink были выполнены и залогированы:

- `scripts/bootstrap_project.m`
- `runtests('tests')`
- `scripts/build_mil_top.m`
- `scripts/run_mil_top_hover.m`
- `scripts/run_mil_top_yaw_step.m`

Raw logs сохранены в:

- `artifacts/logs/task_07_bootstrap.txt`
- `artifacts/logs/task_07_runtests.txt`
- `artifacts/logs/task_07_build_mil_top.txt`
- `artifacts/logs/task_07_run_mil_top_hover.txt`
- `artifacts/logs/task_07_run_mil_top_yaw_step.txt`

## Результаты тестов

`runtests('tests')` выполнен успешно.

Итог:

- выполнено `33` тестовые проверки
- `33/33` passed
- `0` failed
- `0` incomplete

Новые проверки подтвердили:

- `uav.sl.make_bus_defs()` создаёт корректные `Simulink.Bus` definitions
- `scripts/build_mil_top.m` создаёт и обновляет `models/mil_top.slx` без ошибок
- `run_mil_top_hover` выполняется как smoke scenario
- true и estimated quaternion norms остаются близкими к `1`
- thin Simulink shell выдаёт структуры, согласованные с существующим code-centric API
- hover-run через Simulink shell согласуется с `uav.sim.run_case_with_estimator`

## Результаты demo-сценариев

### `run_mil_top_hover`

Получено:

- `final position NED = [0.000000 0.000000 1.186404] m`
- `final altitude = -1.186404 m`
- `final estimated altitude = -1.159642 m`
- `final estimated Euler = [0.000000 0.000000 0.000000] rad`
- `final true quat norm = 1.000000000000`
- `final estimated quat norm = 1.000000000000`

### `run_mil_top_yaw_step`

Получено:

- `final yaw rate = 3.022337 rad/s`
- `final yaw estimate = 2.052485 rad`
- `final altitude = -1.187754 m`
- `final estimated altitude = -1.160946 m`
- `final true quat norm = 1.000000000000`
- `final estimated quat norm = 1.000000000000`

## Ограничения текущего shell

- shell работает только как `MIL`, без `SIL/HIL`
- `.slx` остаётся минимальным orchestration-layer и не покрывает route/control orchestration
- model build пока опирается на base workspace и script-generated bus objects
- нет многочастотного scheduler, rate groups, delay/fault orchestration и расширенного logging policy
- estimator layer остаётся минимальным и не превращается в navigation EKF по `XYZ`

## Почему этот shell считается thin, а не source-of-truth model

Этот shell считается thin, потому что:

- plant propagation выполняется через `uav.sim.plant_step_struct`
- sensor sampling выполняется через `uav.sensors.sensors_step`
- estimator propagation выполняется через `uav.est.estimator_step`
- bus definitions и сам `.slx` создаются из `.m`-скриптов
- `.slx` не содержит ручной блочной реализации rigid-body физики, sensor layer или estimator layer

То есть source of truth по-прежнему остаётся в `.m`-коде и текстовых артефактах, а `models/mil_top.slx` — это только тонкая orchestration shell для MIL-сценариев.
