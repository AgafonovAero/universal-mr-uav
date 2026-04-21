# TASK-25. Приведение ArduPilot JSON к официальной MATLAB-схеме

## Что исправлено относительно прежнего цикла

В TASK-25 основной путь `ArduPilot JSON + MATLAB-модель` приведен к
официальной схеме MATLAB backend ArduPilot:

- MATLAB слушает UDP-порт `9002`;
- один валидный двоичный пакет ArduPilot вызывает ровно один шаг физики;
- один шаг физики вызывает ровно один JSON-ответ;
- `frame_rate` используется для вычисления `dt`;
- `frame_count` используется для учета дублей, пропусков и сброса
  состояния после перезапуска SITL;
- основной JSON-кадр формируется в официальном виде:
  - `timestamp`;
  - `imu.gyro`;
  - `imu.accel_body`;
  - `position`;
  - `velocity`;
  - `attitude`.

`ExternalNav` в TASK-25 не используется как основной путь, поскольку
официальный backend ArduPilot сначала требует корректного JSON lockstep.

## Официальный эталон

В качестве внешнего эталона использованы:

- `libraries/SITL/examples/JSON/MATLAB/SITL_connector.m`;
- `libraries/SITL/examples/JSON/MATLAB/Copter/SIM_multicopter.m`;
- `libraries/SITL/examples/JSON/readme.md`.

Инвентаризация перенесенных правил сохранена в файле:

`artifacts/reports/task_25_ardupilot_matlab_reference_inventory_ru.md`

## Проверка accel_body

Сравнение `accel_body` с официальным MATLAB-примером выполнено для пяти
режимов.

Результат:

- в зависании, при вертикальных ускорениях и при малых углах крена и
  тангажа расхождений не обнаружено;
- максимальное расхождение `9.81 м/с^2` наблюдается только в состоянии
  покоя на земле и связано с отдельным наземным ограничением.

Это означает, что основной путь `accel_body` приведен к официальной
формуле, а оставшееся отличие является осознанным наземным ограничением.

## Выбранный профиль ArduPilot

Используется профиль:

`tools/ardupilot/wsl/task25_official_matlab_json.parm`

Фактически примененные параметры после загрузки:

- `INS_USE = 1`;
- `INS_USE2 = 0`;
- `INS_USE3 = 0`;
- `INS_ENABLE_MASK = 1`;
- `LOG_DISARMED = 1`;
- `SCHED_LOOP_RATE = 150`;
- `AHRS_EKF_TYPE = 10`;
- `GPS1_TYPE = 100`;
- `SIM_MAG1_DEVID = 97539`.

Не считаны по фактическому ответу профиля:

- `GPS_TYPE`;
- `ARMING_CHECK`.

Windows-side helper получил `HEARTBEAT` по каналу:

`udpin:0.0.0.0:14552`

## Официальный lockstep-прогон 20 секунд

Выполнен 20-секундный проверочный прогон в официальном режиме.

Получено:

- `valid_rx_count = 3203`;
- `json_response_tx_count = 3202`;
- `missed_frame_count = 0`;
- `duplicate_frame_count = 0`;
- `last_frame_count = 3202`.

Это подтверждает, что основной контур работает по официальной схеме
`packet -> step -> response`.

## Попытка arm и первого набора тяги

Выполнен отдельный прогон с попыткой:

- дождаться 20 секунд устойчивого обмена;
- перевести аппарат в `GUIDED`;
- выполнить `arm`;
- при успешном взведении перейти к `takeoff 1 m`.

Фактический результат:

- `arm = нет`;
- `COMMAND_ACK = 4`;
- первая причина отказа:
  `Arm: System not initialised`;
- `motor_pwm_us = 0..1000 мкс`;
- `motor_cmd_radps = 0..0 рад/с`;
- рост тяги не подтвержден;
- изменение высоты не подтверждено.

Следовательно, официальный backend уже работоспособен, но следующий
верхний блокер находится в инициализации ArduPilot, а не в transport
layer JSON/UDP.

## Графики

Сформированы графики:

- `artifacts/figures/task_25_lockstep_counts.png`
- `artifacts/figures/task_25_pwm_channels.png`
- `artifacts/figures/task_25_motor_commands.png`
- `artifacts/figures/task_25_accel_body.png`
- `artifacts/figures/task_25_altitude.png`
- `artifacts/figures/task_25_attitude.png`

## Что не является летной валидацией

Полученный результат не является:

- летной валидацией;
- подтверждением устойчивого автоматического полета;
- подтверждением штатного взлета.

Подтвержден только официальный MATLAB backend ArduPilot и установлен
следующий инженерный блокер:

`Arm: System not initialised`

## Следующий этап

Следующий этап должен быть направлен на устранение именно этой причины:

- проверить готовность EKF/AHRS после загрузки `AHRS_EKF_TYPE = 10` и
  `GPS1_TYPE = 100`;
- установить, чего не хватает для перехода ArduPilot в состояние,
  допускающее `arm`;
- после этого повторить `arm` и первый набор тяги без отхода от
  официальной MATLAB-схемы.
