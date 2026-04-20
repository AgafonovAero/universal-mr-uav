# TASK-18. Двухкнопочный стенд ArduPilot SITL с Mission Planner

## Что сделано

В ветке `task/18-two-button-ardupilot-stand` подготовлен practically запускаемый
верхний вход `OneKeyArduPilotStand` с режимами:

- `OneKeyArduPilotStand("stand-check")`
- `OneKeyArduPilotStand("fly-internal", "Execute", true, "GroundStation", "MissionPlanner")`
- `OneKeyArduPilotStand("fly-json-model", "Execute", true, "GroundStation", "MissionPlanner")`
- `OneKeyArduPilotStand("stop", "Execute", true)`

Режим `fly-internal` использует штатную внутреннюю модель движения `ArduPilot SITL`
без MATLAB-модели. Режим `fly-json-model` использует `ArduPilot SITL` с внешней
MATLAB-моделью `universal-mr-uav` через `JSON/UDP`. Режим `stop` останавливает
процессы стенда без оставления фоновых хвостов в `WSL`.

## Результат fly-internal

- `ArduPilot SITL` запущен: да.
- `Mission Planner` запущен: да.
- `HEARTBEAT` программно подтвержден: да, по каналу `tcp:127.0.0.1:5760`.
- Дополнительный поток `MAVLink` на `udp:172.19.208.1:14552` отдельным слушателем
  не подтвержден.
- Команда взведения принята: да.
- Команда `takeoff 5 m` принята: да.
- Изменение высоты подтверждено: да.
- Максимальная относительная высота по журналу: `5.046 м`.
- `ArduPilot` оставался жив в ходе прогона: да.

Источники фактических данных:

- `artifacts/logs/task_18_internal_sitl_start.txt`
- `artifacts/logs/task_18_internal_sitl_takeoff.txt`
- `artifacts/reports/task_18_internal_sitl_demo.csv`

## Результат fly-json-model

Сначала восстановлен базовый обмен TASK-15 в том же режиме запуска `arducopter`
через `JSON:172.19.208.1`, после чего выполнен длительный прогон обмена и попытка
взведения.

### Базовый обмен

- `valid_rx_count`: `561`
- `json_tx_count`: `1705`
- `response_tx_count`: `560`
- `last_frame_count`: `558`
- Процесс `arducopter` после baseline-прогона оставался жив: да.

### Длительный прогон JSON/UDP с MATLAB-моделью

- `valid_rx_count`: `1128`
- `json_tx_count`: `3492`
- `response_tx_count`: `1125`
- `last_frame_count`: `1680`
- Последний `magic`: `18458`
- Последние принятые `ШИМ`: `[1000 1000 1000 1000] us`
- Последние команды частоты вращения винтов: `[0 0 0 0] рад/с`
- Итоговый статус обмена: `устойчивый обмен подтвержден`
- `Mission Planner` запущен: да

### Попытка взведения в режиме с MATLAB-моделью

- Попытка `arm` выполнена: да.
- `HEARTBEAT` по каналу `tcp:127.0.0.1:5763` подтвержден: да.
- `COMMAND_ACK`: `4`
- Взведение выполнено: нет.
- Первая зафиксированная причина отказа по `STATUSTEXT`: `Arm: Accels inconsistent`
- Диапазон `motor_pwm_us` в подтвержденном прогоне: `1000..1000 us`
- Диапазон `motor_cmd_radps` в подтвержденном прогоне: `0..0 рад/с`

Источники фактических данных:

- `artifacts/logs/task_18_json_model_exchange.txt`
- `artifacts/logs/task_18_json_model_arm_attempt.txt`
- `artifacts/logs/task_18_json_model_statustext.txt`
- `artifacts/reports/task_18_json_model_demo.csv`

## Созданные графики

- `artifacts/figures/task_18_internal_altitude.png`
- `artifacts/figures/task_18_internal_mode_arm_state.png`
- `artifacts/figures/task_18_internal_mavlink_status.png`
- `artifacts/figures/task_18_json_exchange_counts.png`
- `artifacts/figures/task_18_json_pwm_channels.png`
- `artifacts/figures/task_18_json_motor_commands.png`
- `artifacts/figures/task_18_json_altitude.png`
- `artifacts/figures/task_18_json_attitude.png`

## Что не является летной валидацией

Настоящий этап не является летной валидацией, не подтверждает устойчивый
автоматический полет и не подтверждает пригодность внешней MATLAB-модели к
штатной эксплуатации без дополнительной диагностики подсистемы инерциальных
данных и условий взведения `ArduPilot`.

## Следующий шаг

Следующий шаг должен быть направлен не на наращивание отчетов, а на устранение
точной причины отказа `Arm: Accels inconsistent` в режиме `fly-json-model`:

- проверить согласованность `imu.accel_body` и ориентации;
- проверить временную синхронизацию и монотонность `timestamp`;
- проверить инициализацию `EKF/AHRS` до первой команды `arm`;
- повторить попытку взведения только после подтвержденного устойчивого обмена.

## Проверки

- `run('scripts/bootstrap_project.m')`: выполнено, журнал сохранен в
  `artifacts/logs/task_18_bootstrap.txt`
- `runtests('tests')`: `72 Passed, 0 Failed, 0 Incomplete`, журнал сохранен в
  `artifacts/logs/task_18_runtests.txt`
