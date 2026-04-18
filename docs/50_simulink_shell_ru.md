# Thin Simulink Shell Stage-1.5+

## Зачем нужен thin shell

На текущем этапе Simulink нужен не как новый инженерный source of truth, а как минимальная orchestration-оболочка поверх уже существующего code-centric ядра.

Это означает:

- математическая модель движения остается в `.m`-коде
- sensor layer остается в `.m`-коде
- estimator layer остается в `.m`-коде
- `.slx` используется только как тонкая оркестрация, логирование и фиксация boundary

## Два shell-сценария после TASK-08

После TASK-08 в репозитории существуют два script-generated shell-сценария:

1. `models/mil_top.slx`
2. `models/sil_top.slx`

Их роли различаются:

- `mil_top.slx` нужен для тонкого MIL orchestration поверх внутреннего code-centric API
- `sil_top.slx` нужен для SIL-prep boundary между будущим внешним flight stack'ом и тем же code-centric ядром

При этом ни один из `.slx` файлов не становится source of truth.

## Почему `.slx` не переносит физику

Любой thin shell в этом репозитории считается допустимым только если:

- plant propagation выполняется через `uav.sim.plant_step_struct`
- sensor sampling выполняется через `uav.sensors.sensors_step`
- estimator propagation выполняется через `uav.est.estimator_step`
- bus definitions создаются через `.m`-функции
- top model создается или пересоздается скриптом

То есть в `.slx` не должно быть ручной block-diagram реализации:

- rigid-body 6DOF физики
- модели ВМГ
- sensor layer
- estimator layer

## MIL shell

MIL shell остается минимальным и совместимым с TASK-07:

- `uav.sl.Stage15MILSystem`
- `uav.sl.make_bus_defs`
- `scripts/build_mil_top.m`
- `scripts/run_mil_top_hover.m`
- `scripts/run_mil_top_yaw_step.m`

TASK-08 не должен ломать этот сценарий.

## SIL-prep shell

TASK-08 добавляет отдельный thin SIL-prep shell:

- `uav.sil.*` фиксирует канонический внешний интерфейс
- `uav.sl.Stage15SILBridgeSystem` связывает внешний actuator packet с существующим plant/sensors/estimator ядром
- `uav.sl.StubExternalFCSSystem` временно играет роль внешнего flight controller для smoke tests
- `uav.sl.make_sil_bus_defs` создает bus objects без `.sldd`
- `scripts/build_sil_top.m` script-generated создает `models/sil_top.slx`

## Почему в `sil_top.slx` есть feedback delay

В `sil_top.slx` допускается один минимальный дискретный feedback delay на пути обратной связи sensor packet -> stub FCS.

Его роль:

- разорвать algebraic loop
- зафиксировать детерминированную дискретную boundary между внешним flight stack и kernel bridge
- не переносить физику, sensor layer или estimator layer в block diagram

Этот delay не меняет архитектурный принцип: source of truth все равно остается в `.m`-коде.

## Что именно делает TASK-08

TASK-08 делает только SIL-prep interface layer:

- задает формат actuator command для внешнего flight stack
- задает формат sensor packet для внешнего flight stack
- задает прозрачный multi-rate scheduler packet boundary
- показывает минимально замкнутую Simulink boundary через `sil_top.slx`

TASK-08 не делает:

- реальный ArduPilot runtime bridge
- реальный PX4 runtime bridge
- MAVLink/UDP transport layer
- HIL integration

## Приоритет следующего roadmap

После фиксации интерфейсной границы дальнейший приоритет такой:

1. ArduPilot SIL
2. PX4 SIL
3. PX4 HIL

Почему следующим шагом логично делать именно ArduPilot SIL:

- boundary внешнего actuator/sensor interface уже зафиксирован
- нужен первый реальный стек-специфичный adapter поверх готовой boundary
- ArduPilot SIL дает ближайшую инженерную верификацию полезности нового interface layer

## Запуск shell-сценариев

Из MATLAB:

```matlab
run('scripts/bootstrap_project.m');

run('scripts/build_mil_top.m');
run('scripts/run_mil_top_hover.m');
run('scripts/run_mil_top_yaw_step.m');

run('scripts/build_sil_top.m');
run('scripts/run_sil_stub_hover.m');
run('scripts/run_sil_stub_yaw_step.m');
```
