# Отчет по TASK-03-RU

## Что сделано

- В `Stage-1 / Stage-1.5` добавлено расчетное ядро ВМГ, агрегатор сил и моментов, минимальный `plant_step`, demo-сценарии и unit-тесты.
- После follow-up замечания P2 исправлен единый источник коэффициентов ротора:
  - добавлен helper `uav.vmg.rotor_coeffs`;
  - `forces_moments_sum`, `mixer_quad_x`, `run_hover_demo` и связанные тесты читают коэффициенты через него;
  - preset `default_params_quad_x250.m` хранит предпочтительный источник в `params.rotor.*`, а legacy-поля оставлены только для совместимости Stage-1.
- Raw logs локальных MATLAB-прогонов сохранены в `artifacts/logs/`.

## Что реально запускалось локально

Дата локального прогона: `2026-04-18`

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/run_motor_spool_demo.m`
4. `scripts/run_openloop_hover_demo.m`

Raw logs:

- `artifacts/logs/task_03_bootstrap.txt`
- `artifacts/logs/task_03_runtests.txt`
- `artifacts/logs/task_03_motor_spool_demo.txt`
- `artifacts/logs/task_03_openloop_hover_demo.txt`

## Фактические результаты локального прогона

- Все 11 тестов из `tests/` прошли успешно.
- `run_motor_spool_demo`:
  - `tau = 0.0500 s`
  - `command = 553.681 rad/s`
  - `final rotor speeds = [553.673 553.673 553.673 553.673] rad/s`
  - `command error = [0.007902 0.007902 0.007902 0.007902] rad/s`
  - `last domega = [0.197560 0.197560 0.197560 0.197560] rad/s^2`
- `run_openloop_hover_demo`:
  - `hover command = [553.681 553.681 553.681 553.681] rad/s`
  - `final rotor speeds = [553.681 553.681 553.681 553.681] rad/s`
  - `total thrust = 9.810000 N`
  - `weight = 9.810000 N`
  - `vertical accel estimate = 7.105427e-15 m/s^2`

## Принятые допущения

- Команда двигателя в этой стадии задается как ссылка по угловой скорости ротора в `rad/s`.
- Для минимального расчетного ядра используется простой дискретный шаг и прозрачная кодовая реализация без Simulink-оболочки.
- Модель `ESC + motor` одинакова для всех четырех роторов и параметризуется одним preset-набором.
- Координатные соглашения `NED` и связанная система координат `X forward, Y right, Z down` сохранены без изменений.

## Ограничения текущей стадии

- Работа ограничена `Stage-1`; GUI, `.slx`, `.mlapp`, `.sldd`, `.prj` не создавались.
- Модель среды по-прежнему включает только гравитацию.
- Сложная аэродинамика, sensor model, расширенные контуры управления и валидация на летных данных вне scope этой задачи.
- Legacy-поля `params.kT_N_per_radps2` и `params.kQ_Nm_per_radps2` пока сохранены только ради совместимости со Stage-1 кодом, но источником истины считаются `params.rotor.*`.
