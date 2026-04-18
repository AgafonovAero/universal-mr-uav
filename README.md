# Universal MR UAV

Минимальное кодо-ориентированное ядро `Stage-1` для универсального многороторного БПЛА в MATLAB.

## Что есть в Stage-1

- математическая модель движения 6DOF на кватернионах
- простая модель ВМГ `T = kT * omega^2`, `Q = kQ * omega^2`
- микшер только для схемы `quad-X`
- модель среды только с гравитацией
- минимальный каркас ПИД-регулятора угловых скоростей
- baseline preset `quad-X 250 mm`
- базовый hover demo
- unit tests для кватернионов, hover-баланса и микшера

## Соглашение по состоянию

`uav.core.eom6dof_quat` использует упакованный 13-мерный вектор состояния:

`x = [p_ned_m; v_b_mps; q_nb; w_b_rps]`

- `p_ned_m` - положение в системе `NED`, м
- `v_b_mps` - линейная скорость в связанной системе координат, м/с
- `q_nb` - scalar-first кватернион поворота из body в `NED`
- `w_b_rps` - угловая скорость в связанной системе координат, рад/с

## Геометрия quad-X

В baseline preset явно разведены три геометрических величины:

- `wheelbase_m` - диагональ motor-to-motor между противоположными роторами
- `motor_radius_m` - расстояние от центра аппарата до каждого ротора
- `motor_xy_m` - матрица `4x2` координат роторов в связанной системе координат, каждая строка имеет вид `[x_i, y_i]`

Для текущего `quad-X` с нумерацией роторов `1` front-left, `2` front-right, `3` rear-right, `4` rear-left:

```text
motor_xy_m =
[
 +x  -y
 +x  +y
 -x  +y
 -x  -y
]
```

В матрице микширования используются физические плечи:

- по крену: `-y_i`
- по тангажу: `x_i`
- по рысканью: `(kQ / kT) * spin_dir(i)`

Это убирает прежнюю двусмысленность и исключает повторное деление на `sqrt(2)`.

## Быстрый старт

Из корня репозитория в MATLAB:

```matlab
run('scripts/bootstrap_project.m');
demo = uav.sim.run_hover_demo();
results = runtests('tests');
table(results)
```

## Структура

```text
docs/
scripts/
src/+uav/+core/
src/+uav/+vmg/
src/+uav/+env/
src/+uav/+ctrl/
src/+uav/+sim/
data/presets/
tests/
artifacts/logs/
artifacts/reports/
```

См. также:

- `docs/00_scope_ru.md`
- `docs/10_frames_and_conventions_ru.md`
- `artifacts/reports/task_02_summary_ru.md`
