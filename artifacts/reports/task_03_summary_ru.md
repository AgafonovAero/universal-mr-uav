# Отчет по TASK-03-RU

## Что было сделано

- `AGENTS.md` дополнен блоками `Git workflow` и `Review guidelines`.
- Создан `TASK_03_RU.md` с формализацией следующего инженерного этапа.
- Baseline preset `default_params_quad_x250.m` расширен параметрами `params.motor.*` и `params.rotor.*` при сохранении legacy-полей для совместимости.
- Добавлена дискретная модель двигателя `uav.vmg.motor_esc_step`.
- Добавлен агрегатор сил и моментов `uav.core.forces_moments_sum`.
- Добавлен минимальный `uav.sim.plant_step` с 17-мерным состоянием.
- Добавлены demo-сценарии:
  - `scripts/run_motor_spool_demo.m`
  - `scripts/run_openloop_hover_demo.m`
- Добавлены тесты:
  - `tests/test_motor_esc_step.m`
  - `tests/test_forces_moments_sum.m`
  - `tests/test_plant_step.m`
- Обновлены `README.md`, `docs/00_scope_ru.md`, `docs/10_frames_and_conventions_ru.md`.
- Сохранены raw logs MATLAB-прогонов в `artifacts/logs/`.

## Принятые допущения

- Команда двигателя в `Stage-1.5` задается непосредственно как ссылка по угловой скорости ротора в рад/с.
- Для минимального plant kernel достаточно дискретного explicit Euler-интегрирования жесткого тела.
- Модель `ESC + motor` одинакова для всех четырех роторов и задается одним набором параметров.
- Система координат и знаковые соглашения из `Stage-1` не менялись.

## Что реально запускалось

1. `scripts/bootstrap_project.m`
2. `runtests('tests')`
3. `scripts/run_motor_spool_demo.m`
4. `scripts/run_openloop_hover_demo.m`

Raw logs:

- `artifacts/logs/task_03_bootstrap.txt`
- `artifacts/logs/task_03_runtests.txt`
- `artifacts/logs/task_03_motor_spool_demo.txt`
- `artifacts/logs/task_03_openloop_hover_demo.txt`

## Полученные результаты

- Все 11 тестов в `tests/` прошли успешно.
- `motor_esc_step` монотонно сходится к команде и корректно насыщается по максимуму.
- `forces_moments_sum` при одинаковых скоростях роторов дает нулевые `Mx`, `My`, `Mz` и силу вдоль `-Z_b`.
- `plant_step` после раскрутки роторов до hover-команды дает вертикальное ускорение, численно близкое к нулю.
- В `run_motor_spool_demo` финальные скорости роторов достигли `553.673 rad/s` при команде `553.681 rad/s`.
- В `run_openloop_hover_demo` получено:
  - `total thrust = 9.810000 N`
  - `weight = 9.810000 N`
  - `vertical accel estimate = 7.105427e-15 m/s^2`

## Ограничения текущей стадии

- Simulink shell не добавлялся.
- GUI и бинарные артефакты не создавались.
- Сложная аэродинамика, sensor model и развитые контуры управления по-прежнему вне scope.
- `plant_step` остается намеренно простым и прозрачным, без усложнения интегратора и без классов.
