# TASK-19. Устранение отказа взведения ArduPilot JSON по акселерометрам

## 1. Что сделано

Выполнены реальные локальные проверки для режима
`ArduPilot SITL + MATLAB-модель universal-mr-uav + Mission Planner`
через `JSON/UDP`:

- восстановлен устойчивый обмен TASK-15 как базовый режим;
- сохранены первые 200 реально отправленных `JSON`-кадров;
- проверены несколько соглашений по `imu.accel_body`;
- диагностирован состав виртуальных ИНС `ArduPilot`;
- выполнены опыты по задержке перед `arm`;
- выполнен отдельный диагностический прогон с профилем одного ИНС;
- построены графики по инерциальным данным, результатам `arm` и
  командам винтомоторной группы.

## 2. Исходная проблема TASK-18

В TASK-18 были подтверждены:

- устойчивый обмен `ArduPilot JSON + MATLAB-модель`;
- `Mission Planner` запущен;
- попытка `arm` выполняется;
- взведение не выполняется;
- подтвержденная причина отказа:
  `Arm: Accels inconsistent`.

Сообщение `Arm: Accels inconsistent` означает отказ по согласованности
акселерометров в `ArduPilot`.

## 3. Базовый устойчивый обмен

Для TASK-19 базовым режимом считался только прогон, в котором одновременно
выполняются:

- `valid_rx_count > 50`
- `response_tx_count > 50`
- `last_frame_count > 0`
- процесс `arducopter` остается жив

Актуальные подтвержденные значения для восстановленного базового обмена:

- `baseline valid_rx_count = 526`
- `baseline response_tx_count = 523`
- `baseline json_tx_count = 1765`
- `baseline last_frame_count = 519`
- `arducopter` жив после 20 секунд: да

## 4. Диагностика первых JSON-кадров

Сценарий:

- `scripts/diagnose_ardupilot_json_imu_consistency.m`

Сохранены:

- `artifacts/logs/task_19_json_imu_consistency.txt`
- `artifacts/reports/task_19_first_json_frames.jsonl`
- `artifacts/reports/task_19_json_imu_consistency.csv`
- `artifacts/reports/task_19_json_imu_consistency.mat`

Подтверждено:

- обязательные поля присутствуют:
  `timestamp`, `imu.gyro`, `imu.accel_body`, `position`, `velocity`,
  `quaternion`;
- `timestamp` возрастает;
- кватернион конечен и нормирован;
- в первых кадрах при основном режиме `json_accel_mode = current`
  передается:
  `imu.accel_body = [0, 0, 0]`.

Практический вывод:

- проблема связана не с потерей обмена;
- проблема локализуется именно в трактовке инерциальных данных,
  а не в транспортном уровне `JSON/UDP`.

## 5. Проверенные варианты accel_body

Сценарий:

- `scripts/run_ardupilot_json_accel_convention_experiments.m`

Проверены варианты:

- `current`
- `specific_force_frd`
- `minus_specific_force_frd`
- `linear_accel_body`
- `zero_on_ground`

Итог по всем вариантам:

- устойчивый обмен подтвержден;
- `COMMAND_ACK = 4`;
- причина отказа одна и та же:
  `Arm: Accels inconsistent`;
- `motor_pwm_us` после попытки `arm`: `1000..1000 us`;
- `motor_cmd_radps` после попытки `arm`: `0..0 рад/с`.

Выбранный основной вариант после TASK-19:

- `json_accel_mode = current`

Причина выбора:

- ни один из диагностических вариантов не дал положительного результата по
  `arm`;
- основной путь оставлен минимальным и воспроизводимым, без фиктивного
  объявления исправления.

Вывод по знаку ускорения:

- проблема простого знака ускорения не подтверждена как основная;
- ни инверсия знака, ни переход к `linear_accel_body`, ни режим
  `zero_on_ground` не устранили отказ взведения.

## 6. Проверка нескольких виртуальных ИНС

Сценарий:

- `scripts/diagnose_ardupilot_ins_instances.m`

Подтверждено:

- `INS_USE = 1`
- `INS_USE2 = 1`
- `INS_USE3 = 1`
- `INS_ENABLE_MASK = 127`
- присутствуют потоки `SCALED_IMU2` и `SCALED_IMU3`

Это подтверждает, что в штатном JSON-режиме `ArduPilot` использует
несколько виртуальных ИНС.

Дополнительно выполнен диагностический профиль одного ИНС:

- `tools/ardupilot/wsl/task19_single_imu_diagnostic.parm`

В этом профиле:

- `INS_USE = 1`
- `INS_USE2 = 0`
- `INS_USE3 = 0`

Результат:

- устойчивый обмен сохранен;
- `COMMAND_ACK = 4`;
- причина отказа осталась той же:
  `Arm: Accels inconsistent`.

Вывод по нескольким ИНС:

- несколько виртуальных ИНС являются подтвержденным фактором режима;
- однако простое отключение `INS_USE2/INS_USE3` не устранило отказ;
- следовательно, проблема нескольких ИНС подтверждена как часть
  диагностики, но не подтверждена как единственная причина.

## 7. Проверка задержки перед arm

Сценарий:

- `scripts/run_ardupilot_json_arm_delay_experiments.m`

Проверены задержки:

- 5 с
- 15 с
- 30 с
- 60 с

Во всех случаях:

- устойчивый обмен сохранялся;
- `COMMAND_ACK = 4`;
- причина отказа оставалась:
  `Arm: Accels inconsistent`.

Вывод:

- дополнительная задержка перед `arm` требовалась для диагностики,
  но не дала устранения отказа.

## 8. Практический результат TASK-19

На момент завершения TASK-19 подтверждено:

- устойчивый `JSON/UDP`-обмен сохранен;
- `valid_rx_count > 50`;
- `response_tx_count > 50`;
- `Mission Planner` может быть запущен;
- попытка `arm` выполняется;
- `COMMAND_ACK = 4`;
- причина отказа воспроизводима:
  `Arm: Accels inconsistent`;
- ненулевой `motor_pwm_us` не появился;
- ненулевые `motor_cmd_radps` не появились.

Итог по целевой задаче:

- штатное устранение отказа `Arm: Accels inconsistent` в TASK-19
  не достигнуто;
- однако отказ локализован на устойчивом обмене, а не на деградации
  транспортного канала.

## 9. Созданные графики

Созданы файлы:

- `artifacts/figures/task_19_accel_body_components.png`
- `artifacts/figures/task_19_accel_norm.png`
- `artifacts/figures/task_19_quaternion_norm.png`
- `artifacts/figures/task_19_arm_result_by_accel_mode.png`
- `artifacts/figures/task_19_pwm_after_fix.png`
- `artifacts/figures/task_19_motor_commands_after_fix.png`

## 10. Что не является летной валидацией

TASK-19 не является:

- летной валидацией;
- подтверждением устойчивого автоматического полета;
- подтверждением пригодности режима `ArduPilot JSON + MATLAB-модель`
  к штатной эксплуатации.

## 11. Следующий этап

Следующим шагом требуется:

1. адресно проверить физический смысл `imu.accel_body` относительно
   внутреннего ожидания `ArduPilot SITL JSON`;
2. собрать дополнительные диагностические данные по внутренним ИНС
   `ArduPilot` без подмены штатного решения отключением всех проверок;
3. подтвердить путь, при котором `COMMAND_ACK` изменится с `4` на
   принятую команду взведения и появятся ненулевые команды
   винтомоторной группы.
