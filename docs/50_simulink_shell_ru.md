# Thin Simulink Shell Stage-1.5+

## Зачем нужен thin Simulink shell

На текущем этапе нужен не новый source-of-truth model, а минимальная orchestration-оболочка для `MIL` поверх уже существующего code-centric ядра:

- математическая модель движения остаётся в `.m`-коде
- sensor layer остаётся в `.m`-коде
- estimator layer остаётся в `.m`-коде
- Simulink нужен только как внешний дискретный scheduler, источник сигналов и слой логирования

Такой подход позволяет:

- сохранить текстовую верифицируемость репозитория
- не переносить физику объекта управления в бинарный `.slx`
- готовить будущие orchestration-сценарии без разрушения kernel-first архитектуры

## Почему source of truth не переносится в `.slx`

`models/mil_top.slx` не является основной инженерной моделью. Он считается thin shell, потому что:

- plant propagation выполняется через `uav.sim.plant_step_struct`
- sensor sampling выполняется через `uav.sensors.sensors_step`
- estimator propagation выполняется через `uav.est.estimator_step`
- bus definitions создаются функцией `uav.sl.make_bus_defs`
- сама top model регенерируется скриптом `scripts/build_mil_top.m`

Иными словами, `.slx` не содержит ручной block-diagram реализации:

- rigid-body 6DOF физики
- модели ВМГ
- sensor layer
- estimator layer

## Состав `uav.sl.Stage15MILSystem`

`uav.sl.Stage15MILSystem` является `matlab.System` wrapper'ом с дискретным шагом `dt_s`.

Публичные свойства:

- `dt_s` - шаг дискретизации
- `state0` - начальное canonical state struct
- `params` - параметрический preset/config struct

Внутреннее runtime-состояние:

- состояние объекта управления `state_internal`
- состояние оценивателя `estimator_internal`
- флаг инициализации estimator layer

На каждом такте блок:

1. берет текущее canonical state struct
2. формирует sampled plant diagnostics из rotor speeds
3. вызывает `uav.sensors.sensors_step`
4. инициализирует или продвигает `uav.est.estimator_step`
5. отдает:
   - `state_out`
   - `sensors_out`
   - `estimator_out`
   - `diag_out`
6. продвигает plant через `uav.sim.plant_step_struct`

Кватернион состояния не нормализуется внутри `.slx` вручную. Нормализация остаётся внутри существующего ядра `uav.sim.plant_step`.

## Bus definitions

Функция `uav.sl.make_bus_defs()` создаёт `Simulink.Bus` objects в base workspace без использования `.sldd`.

Определяются bus'ы для:

- canonical state
- sensor layer
- estimator layer
- diag layer

Вложенные sub-bus'ы повторяют структуру существующего API:

- `imu`, `baro`, `mag`, `gnss`
- `attitude`, `altitude`
- `plant`, `estimator`

## Как генерируется `models/mil_top.slx`

Скрипт `scripts/build_mil_top.m` программно:

- создаёт или пересоздаёт `models/mil_top.slx`
- настраивает модель как `discrete`, `fixed-step`
- добавляет `From Workspace` источник команд на моторы
- добавляет один `MATLAB System` block на основе `uav.sl.Stage15MILSystem`
- добавляет логирование `state_out`, `sensors_out`, `estimator_out`, `diag_out`

Цель скрипта — оставить `.slx` минимальным и воспроизводимым, а не редактируемым вручную как инженерный источник истины.

## Как запускать demo-сценарии

Из MATLAB из корня репозитория:

```matlab
run('scripts/bootstrap_project.m');
run('scripts/build_mil_top.m');
run('scripts/run_mil_top_hover.m');
run('scripts/run_mil_top_yaw_step.m');
```

Профили команд создаются функцией:

```matlab
cmd = uav.sl.make_demo_command_profile('hover', params, dt_s, t_final_s);
cmd = uav.sl.make_demo_command_profile('yaw_step', params, dt_s, t_final_s);
```

## Ограничения текущего MIL-shell

- shell работает только как `MIL`, без `SIL/HIL`
- top model остаётся минимальной и не покрывает route/control orchestration
- нет асинхронных rate groups, scheduler logic и многочастотной архитектуры
- нет ручной блочной реализации физики объекта, sensor layer или estimator layer
- нет `.sldd` и model-based parameter governance
- shell опирается на base workspace bus objects и script-generated model rebuild

## Что логично делать следующим шагом

После этого шага инженерно разумны два направления:

1. расширять thin shell до orchestration-уровня `route/control`, оставляя physics/sensors/estimator в `.m`-ядре
2. готовить hooks для `SIL/HIL`, если orchestration boundary уже достаточно стабилизирована

Оба пути совместимы с текущим принципом: source of truth остаётся в MATLAB-коде и текстовых артефактах, а `.slx` остаётся thin shell.
