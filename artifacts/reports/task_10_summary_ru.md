# TASK-10: ArduPilot JSON SITL adapter scaffold

## Что сделано

В рамках TASK-10 подготовлен новый code-centric пакет
`src/+uav/+ardupilot/` для будущей связи `universal-mr-uav`
с `ArduPilot SITL` через JSON interface.

На этом шаге добавлены:

- `default_json_config.m` с явной фиксацией UDP endpoint placeholders,
  frame conventions, PWM semantics и motor order;
- `inspect_sitl_environment.m` для inventory-проверки локальной среды
  без падения при отсутствии ArduPilot;
- `pack_json_fdm_packet.m` для формирования canonical MATLAB struct
  будущего FDM/JSON boundary;
- `unpack_servo_outputs.m` для нормализации servo/PWM packet;
- `pwm_to_motor_radps.m` для явного PWM -> motor rad/s mapping;
- `make_loopback_servo_packet.m` для fake servo-output packet
  без реального ArduPilot;
- `validate_json_packet.m` для структурной проверки packet fields;
- `uav.sim.run_case_with_ardupilot_loopback` как smoke runner
  по цепочке
  `plant -> sensors -> estimator -> ArduPilot-style packet -> fake PWM -> plant`.

Также добавлены:

- `scripts/inspect_ardupilot_sitl_env.m`
- `scripts/run_ardupilot_loopback_hover.m`
- `scripts/run_ardupilot_loopback_yaw_step.m`
- 5 новых tests для TASK-10
- новая документация `docs/70_ardupilot_json_sitl_ru.md`
- обновленный `README.md`

Важно:
TASK-10 не зависит от PR #7 и не переносит изменения из ветки
`task/08a-atc-mil-bridge`.

## Что запускалось

Локально были реально выполнены обязательные прогоны:

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/inspect_ardupilot_sitl_env.m`
4. `scripts/run_ardupilot_loopback_hover.m`
5. `scripts/run_ardupilot_loopback_yaw_step.m`

Raw logs сохранены в:

- `artifacts/logs/task_10_bootstrap.txt`
- `artifacts/logs/task_10_runtests.txt`
- `artifacts/logs/task_10_inspect_ardupilot_sitl_env.txt`
- `artifacts/logs/task_10_run_ardupilot_loopback_hover.txt`
- `artifacts/logs/task_10_run_ardupilot_loopback_yaw_step.txt`

Все эти raw logs пересохранены как UTF-8 text без BOM и без ANSI/control
characters.

## Готовность локальной среды к реальному ArduPilot SITL

По результатам `inspect_ardupilot_sitl_env.m`:

- `ready for real SITL = no`
- `has WSL = no`
- `has Python = yes`
- `has sim_vehicle.py = no`
- `ArduPilot root = <not found>`

Это означает, что локальная среда пока не готова к следующему реальному
ArduPilot SITL шагу.

На текущей машине не хватает как минимум:

- подтвержденного WSL runtime;
- `sim_vehicle.py` в `PATH`;
- локального ArduPilot checkout,
  если запуск будет делаться из локальной source tree.

## Результаты тестов

По итогам `runtests('tests')`:

- `49 tests passed`
- `failed = 0`
- `incomplete = 0`

Новые TASK-10 tests подтвердили:

- валидность default config;
- наличие обязательных полей в FDM packet;
- корректный PWM mapping для canonical values и saturation;
- возврат непустого log из loopback runner;
- сохранение норм quaternion близко к `1`;
- устойчивую работу inventory-функции без установленного ArduPilot.

## Результаты loopback demos

### Hover loopback smoke

`run_ardupilot_loopback_hover.m` дал следующие финальные значения:

- `final altitude = -2.456768 m`
- `final estimated altitude = -2.428282 m`
- `final motor PWM = [1615 1615 1615 1615] us`
- `final motor command = [553.5 553.5 553.5 553.5] rad/s`
- `quat norms = true=1.0, est=1.0`

Этот сценарий подтверждает,
что pack/unpack boundary и PWM mapping работают численно прозрачно,
но не является утверждением о настроенном closed-loop hover.

### Yaw-step loopback smoke

`run_ardupilot_loopback_yaw_step.m` дал следующие финальные значения:

- `final yaw = 77.416971 deg`
- `final yaw rate = 9.436954 rad/s`
- `final altitude = -2.694160 m`
- `final estimated altitude = -2.676790 m`
- `final motor PWM = [1675 1555 1675 1555] us`
- `quat norms = true=1.0, est=1.0`

Этот сценарий подтверждает,
что synthetic PWM yaw-step действительно проходит через adapter boundary
и вызывает ожидаемую реакцию plant,
но все еще остается smoke-level loopback verification.

## Ограничения

- Реальный UDP JSON bridge пока не реализован.
- Реальный `sim_vehicle.py` не запускался.
- Motor order и PWM semantics еще не выровнены по реальному ArduPilot.
- Arming, takeoff и mode handling уровня SITL runtime пока отсутствуют.
- Loopback packet не является реальной логикой внешнего автопилота.
- TASK-10 не меняет математику plant, sensors или estimator.

## Следующий шаг

Следующим отдельным этапом должен стать уже реальный ArduPilot SITL шаг:

1. подготовить WSL и локальный ArduPilot checkout;
2. обеспечить доступность `sim_vehicle.py`;
3. поднять реальный UDP JSON bridge;
4. выровнять motor order и parameter mapping;
5. после этого запустить первые настоящие SITL smoke runs.

Именно после этого можно будет говорить не о scaffold,
а о реальной внешней integration boundary с ArduPilot.
