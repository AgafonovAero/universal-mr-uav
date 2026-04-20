# TASK-14. Однокнопочный стенд ArduPilot SITL

## Назначение задачи

TASK-14 развивает результаты TASK-12 и TASK-13 и формирует
воспроизводимый установочно-пусковой контур для стенда на базе
`ArduPilot SITL`, математической модели `universal-mr-uav` и наземных
станций управления `Mission Planner` и `QGroundControl`.

Цель этапа состоит не в подтверждении устойчивого автоматического полета
и не в летной валидации. На данном шаге требовалось подготовить единый
MATLAB-сценарий, который:

- проверяет готовность Windows и `WSL`;
- оценивает готовность внешнего каталога `ArduPilot`;
- проверяет наличие `Mission Planner` и `QGroundControl`;
- формирует безопасные сценарии подготовки и пуска;
- ведет исходные журналы прогонов;
- явно отделяет проверочный dry-run от фактического запуска.

## Выполненные изменения

В корне репозитория создан входной сценарий `OneKeyArduPilotStand.m`.
Основная логика размещена в `scripts/onekey_ardupilot_stand.m`.

Сценарий поддерживает режимы:

- `check`;
- `install`;
- `start`;
- `stop`;
- `status`;
- `full`.

Для сопровождения стенда создан пакет `src/+uav/+setup/`, включающий:

- `default_stand_config.m`;
- `check_windows_tools.m`;
- `check_wsl_tools.m`;
- `check_ground_control_stations.m`;
- `run_powershell_script.m`;
- `run_wsl_command.m`;
- `stand_status.m`;
- `print_operator_action.m`.

Созданы или обновлены операторские сценарии Windows:

- `tools/ardupilot/windows/Install-MissionPlanner.ps1`;
- `tools/ardupilot/windows/Install-QGroundControl.ps1`;
- `tools/ardupilot/windows/Start-ArduPilotStand.ps1`;
- `tools/ardupilot/windows/Stop-ArduPilotStand.ps1`;
- `tools/ardupilot/windows/Test-ArduPilotStand.ps1`;
- `tools/ardupilot/windows/Setup-WSLForArduPilot.ps1`;
- `tools/ardupilot/windows/Invoke-ArduPilotWslSetup.ps1`.

Созданы или обновлены сценарии внутри `WSL`:

- `tools/ardupilot/wsl/setup_ardupilot_wsl.sh`;
- `tools/ardupilot/wsl/start_arducopter_json_sitl.sh`;
- `tools/ardupilot/wsl/check_ardupilot_wsl.sh`.

Добавлены MATLAB-сценарии верхнего уровня:

- `scripts/run_ardupilot_full_stand.m`;
- `scripts/check_ground_station_connection.m`.

Обновлена документация:

- `README.md`;
- `docs/73_ardupilot_environment_bootstrap_ru.md`;
- `docs/74_onekey_ardupilot_stand_ru.md`.

Добавлены модульные проверки TASK-14:

- `tests/test_onekey_config.m`;
- `tests/test_ground_station_install_scripts.m`;
- `tests/test_ardupilot_stand_scripts_presence.m`;
- `tests/test_run_ardupilot_full_stand_dryrun.m`.

## Что было реально проверено

Локально были фактически выполнены следующие команды и сценарии:

```matlab
run('scripts/bootstrap_project.m')
runtests('tests')
OneKeyArduPilotStand("check")
OneKeyArduPilotStand("install")
OneKeyArduPilotStand("start")
OneKeyArduPilotStand("status")
OneKeyArduPilotStand("full")
```

Прогоны `install`, `start` и `full` выполнялись в безопасном режиме без
фактической установки и без фактического запуска стенда.

## Что было реально установлено

На данной машине в рамках TASK-14 не выполнялась фактическая установка:

- `WSL` административно не включался;
- дистрибутив `Ubuntu` не устанавливался;
- внешний каталог `ArduPilot` внутри `WSL` не создавался;
- `Mission Planner` не устанавливался;
- `QGroundControl` не переустанавливался.

Следовательно, TASK-14 на данной машине подтвердил корректность
сценариев dry-run и операторских инструкций, но не подтвердил
фактический пуск полного стенда.

## Готовность среды

Состояние среды по итогам режима `check` и режима `status`:

- команда `wsl.exe` обнаружена;
- список установленных дистрибутивов `WSL` пуст;
- целевой дистрибутив `Ubuntu` не обнаружен;
- `Git` в Windows обнаружен;
- `Python` в Windows обнаружен;
- `MATLAB` обнаружен;
- `Git` внутри `WSL` не проверен из-за отсутствия дистрибутива;
- `Python` внутри `WSL` не проверен из-за отсутствия дистрибутива;
- внешний каталог `ArduPilot` внутри `WSL` не обнаружен;
- `sim_vehicle.py` не обнаружен;
- `Mission Planner` не установлен;
- `QGroundControl` установлен;
- механизм `UDP` в `MATLAB` доступен;
- готовность к режиму `install`: да;
- готовность к режиму `start`: нет.

## Что автоматизировано

В рамках TASK-14 автоматизированы:

- единая точка входа в MATLAB;
- сбор сводного статуса стенда;
- безопасный dry-run подготовки `WSL`;
- безопасный dry-run подготовки внешнего каталога `ArduPilot`;
- безопасная проверка наличия `Mission Planner`;
- безопасная проверка наличия `QGroundControl`;
- безопасный dry-run пуска `ArduPilot SITL`;
- формирование JSON-файла состояния стенда;
- формирование исходных журналов прогонов.

## Что требует оператора

На данной машине для перехода к фактическому пуску стенда оператору
необходимо выполнить:

1. Подготовить дистрибутив `Ubuntu`:

```powershell
powershell -ExecutionPolicy Bypass -File "tools/ardupilot/windows/Setup-WSLForArduPilot.ps1" -Execute -DistroName "Ubuntu"
```

2. При необходимости установить `Mission Planner`:

```powershell
powershell -ExecutionPolicy Bypass -File "tools/ardupilot/windows/Install-MissionPlanner.ps1" -Execute
```

3. Подготовить внешний каталог `ArduPilot` внутри `WSL`:

```powershell
powershell -ExecutionPolicy Bypass -File "tools/ardupilot/windows/Invoke-ArduPilotWslSetup.ps1" -Execute -DistroName "Ubuntu"
```

4. После подготовки среды повторить фактический запуск стенда из
`MATLAB` с разрешенным выполнением.

## Итог модульных проверок

По журналу `artifacts/logs/task_14_runtests.txt`:

- всего модульных проверок: 72;
- пройдено: 72;
- ошибок: 0;
- незавершенных: 0.

Это подтверждает:

- наличие всех новых файлов TASK-14;
- наличие обязательных безопасных режимов;
- корректность dry-run полного стенда;
- отсутствие аварийного завершения при неполной внешней среде.

## Результат запуска стенда

Фактический запуск `ArduPilot SITL` в рамках TASK-14 не выполнялся,
поскольку среда не готова к режиму `start`.

Следовательно:

- `ArduPilot SITL` фактически не был запущен;
- двоичный пакет от `ArduPilot` не был получен;
- ответная передача после принятого пакета не выполнялась;
- подключение наземной станции управления не подтверждено.

В режиме dry-run сценарий `start` корректно сформировал:

- порт `MAVLink UDP`: `14550`;
- входной порт `JSON/UDP` для MATLAB: `9002`;
- выходной порт `JSON/UDP` для MATLAB: `9003`;
- операторскую инструкцию для `Mission Planner`:
  `Mission Planner -> UDP -> Connect -> port 14550`.

Для `QGroundControl` зафиксировано, что при наличии пакетов `HEARTBEAT`
станция обычно обнаруживает поток `MAVLink` автоматически.

## Ограничения текущего этапа

TASK-14 имеет следующие ограничения:

- не выполняет летную валидацию;
- не подтверждает устойчивый автоматический полет;
- не изменяет математическую модель движения;
- не изменяет модели `Simulink`;
- не включает исходные тексты `ArduPilot` в репозиторий;
- не подменяет отсутствие фактического запуска фиктивными данными;
- не заявляет получение двоичного пакета, если он реально не получен;
- не заявляет ответную передачу, если не было принятого пакета;
- не заявляет подключение наземной станции управления без фактического
  потока `MAVLink`.

## Вывод

TASK-14 выполнен как этап инженерной автоматизации стенда.
На текущей машине подтверждены:

- корректная работа однокнопочного сценария в режимах проверки;
- корректная работа безопасного режима подготовки среды;
- корректная работа безопасного режима запуска стенда;
- наличие `QGroundControl`;
- отсутствие установленного `Mission Planner`;
- отсутствие дистрибутива `Ubuntu` в `WSL`;
- отсутствие готового внешнего каталога `ArduPilot`;
- полное прохождение модульных проверок TASK-14.

## Следующий этап

Следующим этапом должно стать фактическое завершение подготовки среды:

1. установить или включить `Ubuntu` в `WSL`;
2. подготовить внешний каталог `ArduPilot` внутри `WSL`;
3. подтвердить наличие `sim_vehicle.py`;
4. при необходимости установить `Mission Planner`;
5. выполнить фактический пуск `ArduPilot SITL`;
6. затем повторить ожидание первого двоичного пакета и проверку обмена
   `JSON/UDP`;
7. отдельно подтвердить подключение наземной станции управления по
   `MAVLink`.
