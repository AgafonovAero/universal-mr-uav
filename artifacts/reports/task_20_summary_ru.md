# TASK-20. Диагностика отказа взведения ArduPilot по журналам акселерометров

## 1. Что сделано

Выполнена адресная диагностика режима
`ArduPilot SITL + MATLAB-модель universal-mr-uav + Mission Planner`
по внутренним журналам `ArduPilot`, без повторения TASK-19 теми же
вариантами `accel_body` как основного средства исправления.

В TASK-20 выполнены:

- сбор фактического `DataFlash`-журнала `ArduPilot` в режиме `JSON/UDP`;
- разбор внутренних записей `IMU` и параметров `INS_*`;
- проверка того, применяются ли `INS_USE2`, `INS_USE3` и `INS_ENABLE_MASK`
  после загрузки;
- сравнение `imu.accel_body` из сохраненных `JSON`-кадров с внутренними
  значениями акселерометров `ArduPilot`;
- выбор минимального параметрического исправления по данным журнала;
- повторная попытка `arm` после исправления;
- построение графиков по акселерометрам и командам винтомоторной группы.

## 2. Исходная проблема TASK-19

На входе TASK-20 были подтверждены:

- устойчивый обмен `JSON/UDP`;
- `valid_rx_count > 50`;
- `response_tx_count > 50`;
- воспроизводимая причина отказа взведения:
  `Arm: Accels inconsistent`;
- отсутствие результата от простой замены типа или знака `imu.accel_body`;
- отсутствие результата от задержки перед `arm`;
- отсутствие результата от профиля, где менялись только `INS_USE2/INS_USE3`.

Сообщение `Arm: Accels inconsistent` в `ArduPilot` означает отказ по
согласованности акселерометров и блокирует взведение.

## 3. Где найден DataFlash-журнал

Сценарий:

- `scripts/collect_ardupilot_dataflash_accel_log.m`

Сохраненный артефакт:

- `artifacts/reports/task_20_ardupilot_dataflash.bin`

Исходный путь журнала внутри `WSL`:

- `/home/oaleg/src/ardupilot/ArduCopter/logs/00000052.BIN`

По прогону сбора журнала подтверждено:

- baseline-обмен восстановлен:
  - `baseline valid_rx_count = 513`
  - `baseline response_tx_count = 510`
  - `baseline last_frame_count = 506`
- процесс `arducopter` оставался жив;
- попытка `arm` была выполнена;
- исходная причина отказа в этом прогоне:
  `Arm: Accels inconsistent`.

## 4. Какие экземпляры ИНС и акселерометров фактически активны

Сценарий:

- `scripts/parse_ardupilot_accel_dataflash_log.m`

По `DataFlash` baseline-прогона подтверждено:

- активны экземпляры `IMU0` и `IMU1`;
- `IMU2`/`IMU3` в журнале не подтверждены как активные экземпляры
  акселерометра;
- `INS_USE = 1`;
- `INS_USE2 = 1`;
- `INS_USE3 = 1`;
- `INS_ENABLE_MASK = 127`.

Максимальное различие между акселерометрами:

- за весь интервал: `0.005905`
- в окне перед первой попыткой взведения: `0.005905`

Последние сообщения `MSG` включали:

- `EKF3 IMU0 initialised`
- `EKF3 IMU1 initialised`
- `Arm: Accels inconsistent`
- `PreArm: Gyros inconsistent`

Практический вывод:

- исходный JSON-режим реально работал как минимум с двумя экземплярами ИНС;
- причина была связана не с потерей обмена, а с внутренней конфигурацией
  нескольких активных акселерометров.
- диагностически проблемным экземпляром признан второй активный
  акселерометр `IMU1`, поскольку только после его фактического исключения
  через `INS_ENABLE_MASK = 1` исходное сообщение
  `Arm: Accels inconsistent` исчезло.

## 5. Применились ли INS_USE2/INS_USE3 фактически

Сценарий:

- `scripts/verify_ardupilot_ins_params_after_boot.m`

Проверены профили:

1. `default`
2. `task19_single_imu`
3. `task20_enable_mask`

Фактический результат после загрузки:

- `default`:
  - `INS_USE = 1`
  - `INS_USE2 = 1`
  - `INS_USE3 = 1`
  - `INS_ENABLE_MASK = 127`
- `task19_single_imu`:
  - `INS_USE = 1`
  - `INS_USE2 = 0`
  - `INS_USE3 = 0`
  - `INS_ENABLE_MASK = 127`
- `task20_enable_mask`:
  - `INS_USE = 1`
  - `INS_USE2 = 0`
  - `INS_USE3 = 0`
  - `INS_ENABLE_MASK = 1`

Вывод:

- профиль TASK-19 менял флаги `INS_USE2/INS_USE3`, но не исключал второй
  экземпляр ИНС из реально активной маски;
- фактическое ограничение до одного акселерометра получилось только после
  применения `INS_ENABLE_MASK = 1`.

## 6. Сравнение JSON-ускорений и внутренних значений ArduPilot

Сценарий:

- `scripts/compare_json_accel_with_ardupilot_log.m`

Использованы:

- `artifacts/reports/task_19_first_json_frames.jsonl`
- `artifacts/reports/task_20_accel_instances.csv`
- `artifacts/reports/task_20_ardupilot_dataflash.bin`

Результат:

- в первых 200 `JSON`-кадрах MATLAB-модель передает `imu.accel_body = [0, 0, 0]`;
- внутренние значения `IMU0` и `IMU1` близки к нулю;
- максимальное различие между `JSON accel_body` и внутренними экземплярами:
  `0.004816`;
- максимальное различие норм между экземплярами:
  `0.002645`;
- различие больше `1 м/с²` не подтверждено.

Вывод:

- простая проблема знака ускорения не подтверждена как первичная;
- грубое численное расхождение между `JSON` и внутренним журналом не найдено;
- первичный блокер был связан именно с множественностью активных ИНС, а не
  с простой ошибкой знака `accel_body`.

## 7. Какое исправление выбрано

Выбран диагностический профиль:

- `tools/ardupilot/wsl/task20_single_accel_enable_mask.parm`

Профиль задает:

- `INS_USE = 1`
- `INS_USE2 = 0`
- `INS_USE3 = 0`
- `INS_ENABLE_MASK = 1`
- `LOG_DISARMED = 1`

Причина выбора:

- это первый профиль, который фактически применился после загрузки так, чтобы
  `INS_ENABLE_MASK` стал равен `1`;
- после его применения исходный отказ
  `Arm: Accels inconsistent` исчез;
- новая причина отказа стала другой:
  `Arm: 3D Accel calibration needed`.

Следовательно, источник исходного рассогласования был устранен
не сменой `accel_body`, а фактическим ограничением активной ИНС до одного
экземпляра.

## 8. Результат повторной попытки arm после исправления

Сценарий:

- `scripts/run_ardupilot_json_arm_after_accel_log_fix.m`

Подтверждено:

- устойчивый обмен сохранен:
  - `valid_rx_count = 1040`
  - `response_tx_count = 1039`
- профиль исправления фактически применен: да
- `COMMAND_ACK = 4`
- `arm` не выполнен
- новая первая причина отказа:
  `Arm: 3D Accel calibration needed`
- диапазон `motor_pwm_us`: `1000..1000 us`
- диапазон `motor_cmd_radps`: `0..0 рад/с`

Итог по целевой задаче TASK-20:

- исходное сообщение `Arm: Accels inconsistent` больше не является первым
  блокером после выбранного исправления;
- ненулевой `ШИМ` не появился;
- ненулевые команды частоты вращения винтов не появились;
- полное принятие команды `arm` в TASK-20 не достигнуто.

## 9. Какие графики созданы

Созданы:

- `artifacts/figures/task_20_accel_instances.png`
- `artifacts/figures/task_20_accel_instance_difference.png`
- `artifacts/figures/task_20_json_vs_ardupilot_accel.png`
- `artifacts/figures/task_20_pwm_after_accel_fix.png`
- `artifacts/figures/task_20_motor_commands_after_accel_fix.png`

## 10. Что не является летной валидацией

TASK-20 не является:

- летной валидацией;
- подтверждением устойчивого автоматического полета;
- подтверждением пригодности режима
  `ArduPilot JSON + MATLAB-модель universal-mr-uav`
  к штатной эксплуатации.

## 11. Следующий этап

Следующим шагом требуется:

1. адресно диагностировать причину `Arm: 3D Accel calibration needed`
   для оставшегося экземпляра ИНС;
2. проверить параметры калибровки акселерометра и их фактическое состояние
   после загрузки профиля `INS_ENABLE_MASK = 1`;
3. добиться перехода от `COMMAND_ACK = 4` к принятой команде взведения;
4. подтвердить появление `motor_pwm_us > 1000` и
   `motor_cmd_radps > 0` хотя бы на части каналов;
5. не заявлять летную валидацию до фактического ненулевого воздействия
   на винтомоторную группу.

## 12. Проверки

Выполнено:

- `run('scripts/bootstrap_project.m')`
- `runtests('tests')`
- `run('scripts/collect_ardupilot_dataflash_accel_log.m')`
- `run('scripts/parse_ardupilot_accel_dataflash_log.m')`
- `run('scripts/verify_ardupilot_ins_params_after_boot.m')`
- `run('scripts/compare_json_accel_with_ardupilot_log.m')`
- `run('scripts/run_ardupilot_json_arm_after_accel_log_fix.m')`
- `run('scripts/plot_ardupilot_accel_log_diagnostics.m')`

Итог модульных проверок:

- `72 Passed, 0 Failed, 0 Incomplete`
