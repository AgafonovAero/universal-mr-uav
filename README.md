# Universal MR UAV

Минимальное code-centric ядро `Stage-1 / Stage-1.5` для универсального многороторного БПЛА в MATLAB.

## Что есть в репозитории

- математическая модель движения 6DOF на кватернионах
- простая модель ВМГ `T = kT * omega^2`, `Q = kQ * omega^2`
- геометрически корректный микшер `quad-X`
- дискретная модель `ESC + motor` первого порядка
- агрегатор суммарных сил и моментов от роторов
- минимальный `plant_step` без Simulink shell
- baseline preset `quad-X 250 mm`
- demo-сценарии для раскрутки роторов и открытого hover
- unit tests и raw logs MATLAB-прогонов

## Уровни состояния

Жесткое тело в `uav.core.eom6dof_quat` использует 13-мерное состояние:

`x_rigid = [p_ned_m; v_b_mps; q_nb; w_b_rps]`

Минимальное расчетное ядро `Stage-1.5` использует 17-мерное состояние:

`x_plant = [p_ned_m(3); v_b_mps(3); q_nb(4); w_b_rps(3); omega_m_radps(4)]`

где:

- `p_ned_m` - положение в земной системе координат `NED`
- `v_b_mps` - линейная скорость в связанной системе координат
- `q_nb` - scalar-first кватернион поворота из body в `NED`
- `w_b_rps` - угловая скорость в связанной системе координат
- `omega_m_radps` - скорости четырех роторов

## Геометрия и параметры ВМГ

В baseline preset явно используются:

- `wheelbase_m` - диагональ motor-to-motor между противоположными роторами
- `motor_radius_m` - расстояние от центра аппарата до ротора
- `motor_xy_m` - матрица `4x2` координат роторов `[x_i, y_i]`
- `params.motor.*` - параметры динамики двигателя и ограничений
- `params.rotor.*` - коэффициенты тяги и реактивного момента

Для обратной совместимости сохранены и legacy-поля:

- `kT_N_per_radps2`
- `kQ_Nm_per_radps2`

## Команды двигателя

В `Stage-1.5` команда двигателя задается как вектор целевых угловых скоростей роторов:

`motor_cmd_radps = [omega_1; omega_2; omega_3; omega_4]`

Обновление состояния двигателей выполняется функцией `uav.vmg.motor_esc_step`, после чего силы и моменты агрегируются через `uav.core.forces_moments_sum`, а жесткое тело интегрируется в `uav.sim.plant_step`.

## Быстрый старт

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');
run('scripts/run_motor_spool_demo.m');
run('scripts/run_openloop_hover_demo.m');
results = runtests('tests');
table(results)
```

## Архитектурная граница Stage-1.5

- управление и модель остаются text-first и code-centric
- Simulink shell на этом этапе не используется
- `.slx`, `.mlapp`, `.sldd`, `.prj` не создаются

## Структура

```text
docs/
scripts/
src/+uav/+core/
src/+uav/+vmg/
src/+uav/+env/
src/+uav/+ctrl/
src/+uav/+sim/
tests/
artifacts/logs/
artifacts/reports/
```

См. также:

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `TASK_03_RU.md`
