# TASK-08a: inventory внешнего репозитория `atc_controller`

## Контекст шага

Цель текущего шага не в том, чтобы строить новый MIL-контур с нуля, а в том,
чтобы аккуратно встроить внешний локальный репозиторий контроллера
`D:\WORK\AI_SOLUTION\atc_controller` в уже существующий thin MIL shell
`universal-mr-uav`.

Инвентаризация выполнена локально по содержимому репозитория и проверена
прямым MATLAB-вызовом на этой машине.

## Что найдено в репозитории

### Язык и среда исполнения

- Основной язык: MATLAB `.m`
- Поддерживаемая среда: MATLAB/Simulink
- В репозитории явно есть:
  - code-centric MATLAB-ядро в `src/`
  - Simulink builders/harness в `simulink/`
  - параметрические профили в `params/`
  - собственные plant/tests/scenarios
  - MEX/Protected Model pipeline как опциональный слой, но не как
    единственный способ запуска

Вывод: внешний контроллер не является только бинарным артефактом. У него есть
прямой MATLAB backend, что подходит для честного controller-in-the-loop MIL
шага.

### Точка входа

Для прямого MATLAB-вызова найдены две практически полезные точки входа:

1. `FSW_Simulink_wrapper_step(in, P_in)`
2. `ATC_Simulink_wrapper_cmdvec(cmd_v, q_bn, gyro, dt, limit_in, mot_in)`

Для интеграции в `universal-mr-uav` выбрана именно:

- `FSW_Simulink_wrapper_step`

Причина выбора:

- это top-level wrapper, а не только контур угловой стабилизации;
- он уже оформлен как one-step интерфейс для Simulink/MATLAB;
- он сразу выдает `motor_cmd`;
- он лучше подходит для thin MIL shell, чем более низкоуровневый
  `ATC_Simulink_wrapper_cmdvec`.

### Как инициализируется репозиторий

Найдена явная path-bootstrap функция:

- `setup_paths`

Она добавляет на MATLAB path:

- корень репозитория;
- `src/`
- `src/motors/`
- `src/sensors/`
- `params/`
- `plant/`
- `scripts/`
- `simulink/`
- `mex/`
- `tests/`
- `tests/scenarios/`

### Как задаются входы контроллера

Найден шаблон входной структуры:

- `FSW_make_default_in`

Ожидаемая структура `in` включает как минимум:

- `dt`
- `reset`
- `armed`
- `mode_cmd`
- `stick_bf_xy`
- `wp_xy`
- `takeoff_alt`
- `att_cmd_cd`
- `rate_cmd_cds`
- `thr_in`
- `slew_yaw`
- `pos_ned`
- `vel_ned`
- `q_bn`
- `gyro_rads`
- `ground_contact`
- `rc_ok`
- `gps_ok`
- `of_ok`
- `batt_v`

### Какие данные состояния/датчиков он ожидает

Для выбранной точки входа контроллер ожидает не raw IMU-only поток, а уже
агрегированный one-step input:

- положение в земной системе координат `pos_ned`
- скорость в земной системе координат `vel_ned`
- кватернион `q_bn` формата `[w x y z]`
- угловые скорости `gyro_rads`
- флаги `ground_contact`, `rc_ok`, `gps_ok`, `of_ok`
- командный режим `mode_cmd`
- ручные/траекторные команды в зависимости от режима

Инженерно это означает:

- внешний контроллер в прямом MATLAB-вызове не требует прямого доступа к
  внутренней физике объекта управления из `universal-mr-uav`;
- ему нужен adapter layer, который пакует существующие `uav.sensors.*` и
  `uav.est.*` в его fixed-field input struct.

### Какой формат управляющих воздействий он выдает

Для выбранной точки входа `FSW_Simulink_wrapper_step` выход `out` содержит:

- `motor_cmd` размером `4x1`
- `thr_cmd`
- `roll_cd`
- `pitch_cd`
- `yaw_cd`
- `z_sp`
- `mode_cmd`
- `mode_used`
- `spool_state`
- `spool_desired`
- `G`
- `fs_active`
- `fs_reason`
- `atc_dbg`

Критично:

- `motor_cmd` не является напрямую `omega_m_radps`;
- это нормированный motor-layer actuator output внешнего контроллера.

Следовательно, для универсальной математической модели движения нужен явный
обратный переход:

1. `motor_cmd (actuator 0..1)`
2. `actuator -> normalized thrust` через
   `AP_MotorsMatrix_actuator_to_thrust`
3. `normalized thrust -> normalized omega`
4. `normalized omega -> motor_cmd_radps` в каноническом формате
   `universal-mr-uav`

### Соглашения по системе координат и единицам

Найденные внешние соглашения:

- `pos_ned`, `vel_ned`: NED
- `q_bn`: quaternion body -> NED
- `gyro_rads`: body rates в `rad/s`
- ручные углы: `att_cmd_cd` в centi-deg
- угловые скорости команд: `rate_cmd_cds` в centi-deg/s
- `thr_in`: нормированная тяга `0..1`

Сопоставление с `universal-mr-uav`:

- NED совместим
- body axes `X forward, Y right, Z down` совместимы
- `q_nb` в ядре `universal-mr-uav` по фактической семантике тоже задает
  преобразование body -> NED, поэтому содержательно совместим с `q_bn`
- углы/команды нужно конвертировать явно, без скрытых градусов

## Можно ли вызвать контроллер из MATLAB/Simulink напрямую

Да.

### Прямой MATLAB-вызов

Локально проверено:

```matlab
cd('D:/WORK/AI_SOLUTION/atc_controller');
setup_paths;
in = FSW_make_default_in();
out = FSW_Simulink_wrapper_step(in);
```

И отдельно проверено, что после нескольких итераций в armed/manual режиме
`motor_cmd` становится ненулевым и пригодным для bridge layer.

### Прямой Simulink-вызов

Да, в самом внешнем репозитории есть Simulink-friendly wrappers, bus defs и
builders. Но для текущего шага это не требуется как source of truth.

В `universal-mr-uav` предпочтительнее использовать:

- существующий thin MIL shell;
- один MATLAB System block;
- прямой MATLAB backend вызов внешнего контроллера из adapter layer.

## Какой минимальный adapter layer нужен

Для честной интеграции без переписывания `atc_controller` нужен только
следующий минимум.

### 1. Path/bootstrap adapter

Нужно:

- знать абсолютный путь до `atc_controller`
- вызвать `setup_paths`
- проверить наличие:
  - `FSW_Simulink_wrapper_step`
  - `FSW_make_default_in`
  - `ATC_Params_default`
  - `AP_MotorsMatrix_params_QuadX`
  - `AP_MotorsMatrix_actuator_to_thrust`

### 2. Sensor/estimator packing adapter

Нужно собрать `in` для `FSW_Simulink_wrapper_step` из уже существующих слоев:

- `uav.sensors.sensors_step`
- `uav.est.estimator_step`

Минимальная выбранная схема упаковки:

- `gyro_rads` <- `sensors.imu.gyro_b_radps`
- `q_bn` <- `estimator.q_nb`
- `pos_ned(1:2)` <- `sensors.gnss.pos_ned_m(1:2)`
- `vel_ned(1:2)` <- `sensors.gnss.vel_ned_mps(1:2)`
- `pos_ned(3)` <- `-estimator.alt_m`
- `vel_ned(3)` <- `-estimator.vz_mps`

Это позволяет не дублировать sensor layer и estimator layer внутри `.slx`.

### 3. Command-profile adapter

На текущем шаге для smoke-level MIL достаточно:

- `hover`
- `yaw_step`

Выбран минимальный режим:

- `mode_cmd = 5` (`STABILIZE`)

Смысл:

- roll/pitch удерживаются около нуля;
- `thr_in` задает коллектив;
- `yaw_step` реализуется через `rate_cmd_cds(3)` после заданного времени.

### 4. Actuation unpack adapter

Нужно перевести `out.motor_cmd` внешнего контроллера в канонический формат
`universal-mr-uav`.

Принята минимальная схема:

1. raw actuator `motor_cmd`
2. `AP_MotorsMatrix_actuator_to_thrust`
3. `motor_norm_01 = sqrt(thrust_norm)`
4. канонический packet:
   - `mode = "norm01"`
   - `motor_norm_01`
5. существующая функция `uav.sil.actuator_cmd_to_motor_radps`

### 5. Parameter-alignment adapter

Нужна минимальная согласовка внешних параметров с нашей математической
моделью движения:

- `sim_motor_order_map = [1;2;3;4]` для канонического quad-X порядка
  `universal-mr-uav`
- `MOT_THST_HOVER` перенастраивается по реальной hover-thrust доле,
  вычисленной из `mass`, `gravity`, `kT`, `omega_max`

## Ограничения, найденные по результатам inventory

### Что работает уже сейчас

- прямой MATLAB-вызов контроллера доступен;
- quad-X / 4 motors поддерживаются;
- путь интеграции через adapter layer реалистичен;
- нет необходимости переписывать `atc_controller` ради первого MIL шага.

### Что не стоит подделывать

- `motor_cmd` нельзя считать прямым `omega`;
- не надо встраивать plant/sensor/estimator логику внешнего репозитория в
  `mil_top_atc.slx`;
- не надо строить новый MIL-контур с нуля вместо использования
  `uav.sim.plant_step_struct`, `uav.sensors.sensors_step`,
  `uav.est.estimator_step`.

### Что остается ограничением после этого шага

- интеграция идет через direct MATLAB call, а не через реальный runtime bridge;
- пока поддерживается только quad-X и 4 мотора;
- полнофункциональный AUTO/LOITER/ALT_HOLD контур этого внешнего контроллера
  не является целью данного шага;
- это не SIL и не HIL;
- это промежуточный controller-in-the-loop MIL boundary.

## Итог inventory

Интеграция `atc_controller` в `universal-mr-uav` возможна напрямую и честно,
без переписывания внешнего контроллера.

Минимально достаточный путь:

1. bootstrap path внешнего репозитория;
2. pack из `uav.sensors.*` и `uav.est.*` в `FSW_Simulink_wrapper_step`;
3. unpack `motor_cmd` в канонический actuator packet;
4. thin Simulink wrapper поверх уже существующего `.m`-ядра.

Именно этот путь и реализуется в TASK-08a.
