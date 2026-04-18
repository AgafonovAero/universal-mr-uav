function [scheduler_next, valid_flags] = update_rate_scheduler( ...
        scheduler_prev, dt_s, rates_cfg)
%UPDATE_RATE_SCHEDULER Advance one deterministic SIL sensor-rate scheduler.
% Description:
%   Implements a transparent counter-based multi-rate scheduler for the
%   external sensor packet. IMU is emitted every base step, while slower
%   sensors follow explicit decimation counters.
%
% Inputs:
%   scheduler_prev - previous scheduler state struct, or [] for init
%   dt_s           - base discrete step [s]
%   rates_cfg      - scalar struct with base_dt_s, baro_period_s,
%                    mag_period_s, and gnss_period_s
%
% Outputs:
%   scheduler_next - updated scheduler state struct
%   valid_flags    - scalar struct with channel valid flags for this step
%
% Units:
%   seconds for time fields, dimensionless counters otherwise
%
% Assumptions:
%   All slower-channel periods are integer multiples of the base step.

validateattributes(dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, 'dt_s');
rates_cfg = local_validate_rates_cfg(dt_s, rates_cfg);

if isempty(scheduler_prev)
    scheduler_prev = local_make_initial_state(rates_cfg);
else
    scheduler_prev = local_validate_scheduler_state(scheduler_prev, rates_cfg);
end

valid_flags = struct();
valid_flags.imu_valid = true;
valid_flags.baro_valid = (scheduler_prev.baro_counter == 0);
valid_flags.mag_valid = (scheduler_prev.mag_counter == 0);
valid_flags.gnss_valid = (scheduler_prev.gnss_counter == 0);

scheduler_next = scheduler_prev;
scheduler_next.step_index = scheduler_prev.step_index + 1;
scheduler_next.imu_counter = local_advance_counter( ...
    scheduler_prev.imu_counter, scheduler_prev.imu_decimation);
scheduler_next.baro_counter = local_advance_counter( ...
    scheduler_prev.baro_counter, scheduler_prev.baro_decimation);
scheduler_next.mag_counter = local_advance_counter( ...
    scheduler_prev.mag_counter, scheduler_prev.mag_decimation);
scheduler_next.gnss_counter = local_advance_counter( ...
    scheduler_prev.gnss_counter, scheduler_prev.gnss_decimation);
end

function rates_cfg = local_validate_rates_cfg(dt_s, rates_cfg)
%LOCAL_VALIDATE_RATES_CFG Validate the canonical scheduler-rate config.
% Description:
%   Checks that the scheduler configuration uses positive sample periods
%   that are integer multiples of the base step.
%
% Inputs:
%   dt_s      - base discrete step [s]
%   rates_cfg - scalar config struct
%
% Outputs:
%   rates_cfg - normalized config struct
%
% Units:
%   seconds
%
% Assumptions:
%   IMU rate equals the base step.

if ~isstruct(rates_cfg) || ~isscalar(rates_cfg)
    error('uav:sil:update_rate_scheduler:RatesType', ...
        'Expected rates_cfg to be a scalar struct.');
end

required_fields = {'base_dt_s', 'baro_period_s', 'mag_period_s', ...
    'gnss_period_s'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(rates_cfg, field_name)
        error('uav:sil:update_rate_scheduler:MissingRatesField', ...
            'Expected rates_cfg.%s to be present.', field_name);
    end
end

validateattributes(rates_cfg.base_dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'rates_cfg.base_dt_s');
if abs(rates_cfg.base_dt_s - dt_s) > 1.0e-12
    error('uav:sil:update_rate_scheduler:BaseStepMismatch', ...
        'Expected rates_cfg.base_dt_s to match dt_s.');
end

rates_cfg.imu_period_s = dt_s;
rates_cfg.imu_decimation = 1;
rates_cfg.baro_decimation = local_period_to_decimation( ...
    rates_cfg.baro_period_s, dt_s, 'rates_cfg.baro_period_s');
rates_cfg.mag_decimation = local_period_to_decimation( ...
    rates_cfg.mag_period_s, dt_s, 'rates_cfg.mag_period_s');
rates_cfg.gnss_decimation = local_period_to_decimation( ...
    rates_cfg.gnss_period_s, dt_s, 'rates_cfg.gnss_period_s');
end

function decimation = local_period_to_decimation(period_s, dt_s, name)
%LOCAL_PERIOD_TO_DECIMATION Convert one period to an integer decimation.
% Description:
%   Ensures that one sensor period is a positive integer multiple of the
%   base discrete step.
%
% Inputs:
%   period_s - sensor period [s]
%   dt_s     - base discrete step [s]
%   name     - field name for diagnostics
%
% Outputs:
%   decimation - positive integer decimation factor
%
% Units:
%   seconds in, dimensionless out
%
% Assumptions:
%   dt_s is already validated as positive.

validateattributes(period_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, mfilename, name);

decimation = round(period_s / dt_s);
if decimation < 1 || abs((decimation * dt_s) - period_s) > 1.0e-12
    error('uav:sil:update_rate_scheduler:PeriodGrid', ...
        'Expected %s to be an integer multiple of dt_s.', name);
end
end

function scheduler = local_make_initial_state(rates_cfg)
%LOCAL_MAKE_INITIAL_STATE Build the initial counter state.
% Description:
%   Seeds all counters so every channel emits a valid sample on the first
%   scheduler call.
%
% Inputs:
%   rates_cfg - validated rate config
%
% Outputs:
%   scheduler - initialized scheduler state struct
%
% Units:
%   seconds for base_dt_s, dimensionless otherwise
%
% Assumptions:
%   The validated config already carries decimation fields.

scheduler = struct();
scheduler.base_dt_s = rates_cfg.base_dt_s;
scheduler.step_index = 0;
scheduler.imu_decimation = rates_cfg.imu_decimation;
scheduler.baro_decimation = rates_cfg.baro_decimation;
scheduler.mag_decimation = rates_cfg.mag_decimation;
scheduler.gnss_decimation = rates_cfg.gnss_decimation;
scheduler.imu_counter = 0;
scheduler.baro_counter = 0;
scheduler.mag_counter = 0;
scheduler.gnss_counter = 0;
end

function scheduler = local_validate_scheduler_state(scheduler, rates_cfg)
%LOCAL_VALIDATE_SCHEDULER_STATE Validate one scheduler-state struct.
% Description:
%   Checks that the state struct contains the expected counter fields and
%   stays consistent with the provided rate configuration.
%
% Inputs:
%   scheduler - scheduler state struct
%   rates_cfg - validated rate config
%
% Outputs:
%   scheduler - validated scheduler state struct
%
% Units:
%   seconds for base_dt_s, dimensionless otherwise
%
% Assumptions:
%   The state was produced by a previous scheduler call.

if ~isstruct(scheduler) || ~isscalar(scheduler)
    error('uav:sil:update_rate_scheduler:SchedulerType', ...
        'Expected scheduler_prev to be a scalar struct.');
end

required_fields = {'base_dt_s', 'step_index', 'imu_decimation', ...
    'baro_decimation', 'mag_decimation', 'gnss_decimation', ...
    'imu_counter', 'baro_counter', 'mag_counter', 'gnss_counter'};
for k = 1:numel(required_fields)
    field_name = required_fields{k};
    if ~isfield(scheduler, field_name)
        error('uav:sil:update_rate_scheduler:MissingSchedulerField', ...
            'Expected scheduler_prev.%s to be present.', field_name);
    end
end

validateattributes(scheduler.base_dt_s, {'numeric'}, ...
    {'real', 'scalar', 'finite', 'positive'}, ...
    mfilename, 'scheduler_prev.base_dt_s');
if abs(scheduler.base_dt_s - rates_cfg.base_dt_s) > 1.0e-12
    error('uav:sil:update_rate_scheduler:SchedulerBaseStepMismatch', ...
        'Expected scheduler_prev.base_dt_s to match rates_cfg.base_dt_s.');
end

validateattributes(scheduler.step_index, {'numeric'}, ...
    {'real', 'scalar', 'integer', 'finite', 'nonnegative'}, ...
    mfilename, 'scheduler_prev.step_index');

local_validate_counter_pair(scheduler, 'imu', rates_cfg.imu_decimation);
local_validate_counter_pair(scheduler, 'baro', rates_cfg.baro_decimation);
local_validate_counter_pair(scheduler, 'mag', rates_cfg.mag_decimation);
local_validate_counter_pair(scheduler, 'gnss', rates_cfg.gnss_decimation);
end

function local_validate_counter_pair(scheduler, prefix, expected_decimation)
%LOCAL_VALIDATE_COUNTER_PAIR Validate one decimation/counter pair.
% Description:
%   Checks one sensor-rate decimation and its current countdown state.
%
% Inputs:
%   scheduler           - scheduler state struct
%   prefix              - channel prefix string
%   expected_decimation - decimation from the validated rate config
%
% Outputs:
%   none
%
% Units:
%   dimensionless
%
% Assumptions:
%   The struct carries fields "<prefix>_decimation" and "<prefix>_counter".

decimation_field = [prefix '_decimation'];
counter_field = [prefix '_counter'];

validateattributes(scheduler.(decimation_field), {'numeric'}, ...
    {'real', 'scalar', 'integer', 'finite', 'positive'}, ...
    mfilename, ['scheduler_prev.' decimation_field]);
if scheduler.(decimation_field) ~= expected_decimation
    error('uav:sil:update_rate_scheduler:SchedulerDecimationMismatch', ...
        'Expected scheduler_prev.%s to match rates_cfg.', decimation_field);
end

validateattributes(scheduler.(counter_field), {'numeric'}, ...
    {'real', 'scalar', 'integer', 'finite', 'nonnegative'}, ...
    mfilename, ['scheduler_prev.' counter_field]);
if scheduler.(counter_field) >= scheduler.(decimation_field)
    error('uav:sil:update_rate_scheduler:CounterRange', ...
        'Expected scheduler_prev.%s < scheduler_prev.%s.', ...
        counter_field, decimation_field);
end
end

function counter_next = local_advance_counter(counter_prev, decimation)
%LOCAL_ADVANCE_COUNTER Advance one countdown-style decimation counter.
% Description:
%   Emits a zero counter when a sample is due and reloads the counter to
%   decimation-1 after that sample has been emitted.
%
% Inputs:
%   counter_prev - previous countdown state
%   decimation   - positive integer decimation factor
%
% Outputs:
%   counter_next - updated countdown state
%
% Units:
%   dimensionless
%
% Assumptions:
%   counter_prev is already validated to lie in [0, decimation-1].

if decimation == 1
    counter_next = 0;
elseif counter_prev == 0
    counter_next = decimation - 1;
else
    counter_next = counter_prev - 1;
end
end
