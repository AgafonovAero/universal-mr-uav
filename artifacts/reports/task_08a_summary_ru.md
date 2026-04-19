# TASK-08a: ATC MIL bridge summary

## Что реально найдено в `atc_controller`

По inventory и локальной проверке найдено:

- репозиторий реализован в MATLAB `.m` с Simulink-friendly оболочками;
- есть прямой path-bootstrap через `setup_paths`;
- есть прямой top-level MATLAB entrypoint:
  - `FSW_Simulink_wrapper_step(in, P_in)`
- есть fixed-field шаблон входов:
  - `FSW_make_default_in`
- есть motor-layer helpers для обратного перехода из actuator domain:
  - `AP_MotorsMatrix_params_QuadX`
  - `AP_MotorsMatrix_actuator_to_thrust`

Итог:

- прямой вызов внешнего контроллера из MATLAB/Simulink возможен;
- переписывать внешний репозиторий для этого шага не потребовалось.

## Что удалось интегрировать

В `universal-mr-uav` добавлены:

- `src/+uav/+atc/`
- `uav.atc.default_atc_bridge_config`
- `uav.atc.ensure_atc_controller_on_path`
- `uav.atc.make_controller_context`
- `uav.atc.pack_sensor_packet_for_atc`
- `uav.atc.unpack_atc_actuation`
- `uav.sl.make_atc_bus_defs`
- `uav.sl.Stage15ATCMILSystem`
- `scripts/build_mil_top_atc.m`
- `scripts/run_mil_atc_hover.m`
- `scripts/run_mil_atc_yaw_step.m`
- `models/mil_top_atc.slx`
- документация и тесты TASK-08a

Интеграция выполнена поверх существующего `.m`-ядра:

- `uav.sim.plant_step_struct`
- `uav.sensors.sensors_step`
- `uav.est.estimator_step`

`mil_top/slx`, `sil_top/slx`, существующие сценарии и тесты сохранены и
не сломаны.

## Что не удалось интегрировать напрямую

Напрямую не удалось получить устойчивый полноценный hover-containment на
базовых параметрах `atc_controller` для текущей математической модели
движения `universal-mr-uav`.

Фактически локальные demo-прогоны показывают:

- direct-call bridge работает;
- controller output проходит через adapter layer и реально двигает объект
  управления;
- но hover/yaw-step на текущем шаге остаются smoke-level boundary demos,
  а не валидированной замкнутой системой автоматического управления.

Причина ограничения:

- внешний контроллер не отстроен под текущий baseline airframe
  `universal-mr-uav`;
- на boundary уровне выполнена только минимальная параметрическая
  согласовка (`quad-X order`, `hover thrust fraction`);
- полноценная настройка контуров, режимов касания/взлета и airframe profile
  не входила в scope этого шага.

## Какие adapter boundary введены

Введены явные boundary-слои:

1. Path/bootstrap boundary
   - `uav.atc.ensure_atc_controller_on_path`

2. Sensor/estimator packing boundary
   - `uav.atc.pack_sensor_packet_for_atc`

3. External controller context boundary
   - `uav.atc.make_controller_context`

4. Actuation unpack boundary
   - `uav.atc.unpack_atc_actuation`

5. Thin Simulink orchestration boundary
   - `uav.sl.Stage15ATCMILSystem`
   - `models/mil_top_atc.slx`

Ключевой принцип соблюден:

- физика объекта управления, sensor layer и estimator layer не были
  перенесены в `.slx`.

## Что реально запускалось локально

Сохранены raw logs:

- `artifacts/logs/task_08a_bootstrap.txt`
- `artifacts/logs/task_08a_runtests.txt`
- `artifacts/logs/task_08a_build_mil_top_atc.txt`
- `artifacts/logs/task_08a_run_mil_atc_hover.txt`
- `artifacts/logs/task_08a_run_mil_atc_yaw_step.txt`

Локально выполнены:

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/build_mil_top_atc.m`
4. `scripts/run_mil_atc_hover.m`
5. `scripts/run_mil_atc_yaw_step.m`

## Краткий итог по тестам

По `artifacts/logs/task_08a_runtests.txt`:

- полный `runtests('tests')` завершился успешно;
- пройдено `42` теста;
- `failed = 0`
- `incomplete = 0`

Новые тесты TASK-08a проверяют:

- наличие и содержательность inventory;
- согласованность pack/unpack bridge функций;
- программную сборку `models/mil_top_atc.slx`;
- smoke-level выполнение `run_mil_atc_hover`.

## Итог demo-прогонов

### `run_mil_atc_hover`

По `artifacts/logs/task_08a_run_mil_atc_hover.txt`:

- final true position NED: `[-0.005106 -0.000000 8.320881]`
- final true altitude: `-8.320881 m`
- final estimated altitude: `-8.110030 m`
- final ATC motor norm: `[0.722105 0.722105 0.485293 0.485293]`
- final motor command: `[649.894418 649.894492 436.763337 436.763254] rad/s`
- final spool state: `3`

Интерпретация:

- bridge работает;
- controller реально управляет ВМГ;
- но hover на текущем baseline не является устойчиво отстроенным.

### `run_mil_atc_yaw_step`

По `artifacts/logs/task_08a_run_mil_atc_yaw_step.txt`:

- final estimated yaw: `0.735861 rad`
- final true yaw rate: `0.729136 rad/s`
- final true altitude: `-8.320941 m`
- final estimated altitude: `-8.110113 m`
- final ATC motor norm: `[0.685768 0.715942 0.493527 0.536162]`
- final spool state: `3`

Интерпретация:

- yaw-step boundary выполняется;
- внешний контроллер реально отдает различающиеся моторные команды;
- но вертикальный контур для данного airframe profile пока не доведен до
  полноценной верификации.

## Что следующим шагом логично делать дальше

Для следующего инженерного шага логично:

1. Сделать отдельный airframe-tuning pass для `atc_controller` под
   параметры `universal-mr-uav`.
2. Явно согласовать ground-contact / takeoff / throttle semantics между
   двумя репозиториями.
3. Добавить отдельный controller-param preset для baseline quad-X
   `universal-mr-uav`.
4. После этого переходить к следующему stack-oriented шагу:
   - либо более честный ATC MIL closed-loop validation,
   - либо уже конкретный SIL adapter/runtime boundary.

Итог TASK-08a:

- direct интеграция внешнего `atc_controller` в существующий thin MIL shell
  реально выполнена;
- архитектурные ограничения репозитория сохранены;
- получен честный bridge scaffold и smoke-level integration boundary без
  подделки устойчивого closed-loop результата там, где его пока нет.
