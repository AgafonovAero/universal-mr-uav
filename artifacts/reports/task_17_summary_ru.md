# TASK-17. Восстановление публикации артефактов диагностики взведения ArduPilot

## Назначение

Настоящий отчет подготовлен для восстановления публикации результатов TASK-17
в запросе на слияние `PR #17`.

При локальной проверке выяснилось, что часть файлов TASK-17 была не просто
некорректно отображена на GitHub, а реально имела нулевой размер в рабочем
дереве ветки `task/17-ardupilot-arm-rate-fix`.

В рамках текущей работы была выполнена:

- проверка размеров файлов;
- повторная локальная генерация журналов и отчетов TASK-17;
- повторное измерение частоты обмена `ArduPilot SITL` и `MATLAB`;
- повторная серия опытов по взведению;
- восстановление файлов `CSV`, `MAT`, `PNG`, текстового отчета и документа
  `docs/76_ardupilot_arming_rate_diagnostics_ru.md`.

## Какие файлы были локально пустыми

При первичной диагностике нулевой размер имели:

- `artifacts/logs/task_17_arm_rate_experiments.txt`
- `artifacts/logs/task_17_bootstrap.txt`
- `artifacts/logs/task_17_case_a_start.txt`
- `artifacts/logs/task_17_case_b_start.txt`
- `artifacts/logs/task_17_case_c_start.txt`
- `artifacts/logs/task_17_exchange_rate_measurement.txt`
- `artifacts/logs/task_17_runtests.txt`
- `artifacts/logs/task_17_start_exchange_measurement.txt`
- `artifacts/reports/task_17_arm_rate_experiments.csv`
- `artifacts/reports/task_17_exchange_rate_measurement.csv`
- `artifacts/reports/task_17_summary_ru.md`
- `docs/76_ardupilot_arming_rate_diagnostics_ru.md`
- `scripts/measure_ardupilot_json_exchange_rate.m`
- `scripts/run_ardupilot_arm_rate_experiments.m`
- `scripts/plot_ardupilot_arm_rate_experiments.m`
- `src/+uav/+ardupilot/summarize_live_backend_metrics.m`

После восстановления перечисленные текстовые файлы имеют ненулевой размер и
сохраняются в кодировке UTF-8.

## Повторный прогон измерения частоты обмена

Для повторного измерения использовалась конфигурация:

- адрес Windows-хоста для `WSL2`: `172.19.208.1`;
- базовый файл параметров: `tools/ardupilot/wsl/task16_arducopter_loop_rate.parm`;
- целевая частота цикла `MATLAB`: `120 Гц`;
- запуск `ArduPilot SITL` через существующий сценарий
  `tools/ardupilot/windows/Start-ArduPilotJsonSitl.ps1`.

Повторный прогон дал следующие значения:

- `PID arducopter = 395`;
- процесс `arducopter` после запуска подтвержден;
- `valid_rx_count = 1`;
- `json_tx_count = 1792`;
- `response_tx_count = 0`;
- `udp_rx_datagram_count = 1`;
- средняя частота входящих валидных двоичных пакетов: `0.0000 Гц`;
- средняя частота исходящих строк `JSON`: `89.6058 Гц`;
- средняя частота ответных передач после принятого пакета: `0.0000 Гц`;
- приращение `frame_count = 0`;
- последний `frame_count = 0`;
- последний `magic = 18458`;
- последний адрес отправителя двоичного пакета: `172.19.219.165`;
- последний порт отправителя двоичного пакета: `47346`;
- итоговый статус: `непрерывный обмен не подтвержден; принят только единичный двоичный пакет`.

Таким образом, при повторном восстановительном прогоне в репозитории удалось
подтвердить только единичный прием двоичного пакета и исходящий поток строк
`JSON`, но не удалось повторно подтвердить устойчивый ответный обмен.

## Повторная серия опытов по взведению

Были повторно выполнены три опыта:

### Опыт A

- конфигурация: `task16_arducopter_loop_rate.parm`;
- целевая частота цикла `MATLAB`: `120 Гц`;
- `PID arducopter = 457`;
- `valid_rx_count = 1`;
- `json_tx_count = 1829`;
- `response_tx_count = 0`;
- `arm_succeeded = false`;
- процесс `arducopter` после опыта не подтвержден как живой;
- зарегистрированная причина: `[Errno 104] Connection reset by peer`.

### Опыт B

- конфигурация: `task17_loop_rate_50.parm`;
- целевая частота цикла `MATLAB`: `160 Гц`;
- `PID arducopter = 405`;
- `valid_rx_count = 1`;
- `json_tx_count = 1862`;
- `response_tx_count = 0`;
- `arm_succeeded = false`;
- процесс `arducopter` после опыта не подтвержден как живой;
- зарегистрированная причина: `[Errno 104] Connection reset by peer`.

### Опыт C

- конфигурация: `task17_loop_rate_30.parm`;
- целевая частота цикла `MATLAB`: `160 Гц`;
- `PID arducopter = 428`;
- `valid_rx_count = 1`;
- `json_tx_count = 1864`;
- `response_tx_count = 0`;
- `arm_succeeded = false`;
- процесс `arducopter` после опыта не подтвержден как живой;
- зарегистрированная причина: `[Errno 104] Connection reset by peer`.

## Итог по взведению

Повторными опытами взведение `ArduPilot` не подтверждено.

На текущем восстановительном прогоне не удалось воспроизвести ранее заявленный
устойчивый поток входящих двоичных пакетов. Поэтому основной вывод TASK-17 в
обновленных артефактах сформулирован осторожно:

- публикация файлов TASK-17 восстановлена;
- файлы отчетов, сценариев и графиков теперь непустые;
- повторный локальный прогон в текущем состоянии стенда не подтверждает
  устойчивый обмен и не подтверждает взведение;
- первая установленная причина отказа серии опытов: разрыв соединения
  `[Errno 104] Connection reset by peer` после единичного приема пакета.

## Результаты проверок

- `scripts/bootstrap_project.m` выполнен, журнал сохранен в
  `artifacts/logs/task_17_bootstrap.txt`;
- `runtests('tests')` выполнен, журнал сохранен в
  `artifacts/logs/task_17_runtests.txt`;
- итог `runtests('tests')`: `72 Passed, 0 Failed, 0 Incomplete`;
- проверка скрытых и управляющих символов пройдена;
- модели `Simulink` в diff отсутствуют.

## Ограничения

Настоящий отчет не заявляет:

- летную валидацию;
- устойчивый автоматический полет;
- подтвержденное взведение `ArduPilot`;
- подтвержденный управляемый расчетный прогон винтомоторной группы.

## Следующий шаг

Следующим инженерным шагом следует считать не продолжение расширения TASK-17,
а отдельную диагностику причины разрыва соединения `Connection reset by peer`
после первого принятого пакета `JSON/UDP` при повторных прогонах ветки.
