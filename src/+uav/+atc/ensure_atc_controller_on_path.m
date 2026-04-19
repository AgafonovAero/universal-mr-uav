function info = ensure_atc_controller_on_path(cfg)
%ENSURE_ATC_CONTROLLER_ON_PATH Add the external ATC repository to path.
% Description:
%   Bootstraps the MATLAB path for the external `atc_controller`
%   repository and verifies that the direct-call entrypoints required by
%   the MIL bridge are available.
%
% Inputs:
%   cfg - scalar bridge configuration struct from
%         `uav.atc.default_atc_bridge_config`
%
% Outputs:
%   info - struct with resolved repository metadata and availability flags
%
% Units:
%   not applicable
%
% Assumptions:
%   The external repository is available locally on the same machine.

cfg = local_validate_cfg(cfg);
repo_root = char(cfg.external_repo_root);

if exist(repo_root, 'dir') ~= 7
    error('uav:atc:ensure_atc_controller_on_path:MissingRepo', ...
        'External ATC repository is missing: %s', repo_root);
end

addpath(repo_root);

if logical(cfg.enable_external_path_bootstrap)
    if exist(cfg.setup_function, 'file') ~= 2
        error('uav:atc:ensure_atc_controller_on_path:MissingSetupFunction', ...
            'Expected setup function "%s" under %s.', ...
            cfg.setup_function, repo_root);
    end
    feval(cfg.setup_function);
else
    addpath(fullfile(repo_root, 'src'));
    addpath(fullfile(repo_root, 'src', 'motors'));
    addpath(fullfile(repo_root, 'src', 'sensors'));
    addpath(fullfile(repo_root, 'params'));
    addpath(fullfile(repo_root, 'plant'));
    addpath(fullfile(repo_root, 'scripts'));
    addpath(fullfile(repo_root, 'simulink'));
end

required_functions = { ...
    cfg.controller_step_function; ...
    cfg.controller_input_factory; ...
    cfg.controller_param_factory; ...
    cfg.controller_mixer_factory; ...
    cfg.controller_actuator_to_thrust_function};

for k = 1:numel(required_functions)
    if exist(required_functions{k}, 'file') ~= 2
        error('uav:atc:ensure_atc_controller_on_path:MissingFunction', ...
            'Expected external function "%s" to be available on path.', ...
            required_functions{k});
    end
end

info = struct();
info.repo_root = repo_root;
info.direct_call_available = true;
info.setup_function = string(cfg.setup_function);
info.controller_step_function = string(cfg.controller_step_function);
info.required_functions = string(required_functions(:));
end

function cfg = local_validate_cfg(cfg)
%LOCAL_VALIDATE_CFG Validate the ATC bridge configuration struct.

if ~isstruct(cfg) || ~isscalar(cfg)
    error('uav:atc:ensure_atc_controller_on_path:CfgType', ...
        'Expected cfg to be a scalar struct.');
end

required_fields = {'external_repo_root', 'setup_function', ...
    'controller_step_function', 'controller_input_factory', ...
    'controller_param_factory', 'controller_mixer_factory', ...
    'controller_actuator_to_thrust_function', ...
    'enable_external_path_bootstrap'};

for k = 1:numel(required_fields)
    if ~isfield(cfg, required_fields{k})
        error('uav:atc:ensure_atc_controller_on_path:MissingCfgField', ...
            'Expected cfg.%s to be present.', required_fields{k});
    end
end

if ~(ischar(cfg.external_repo_root) || ...
        (isstring(cfg.external_repo_root) && isscalar(cfg.external_repo_root)))
    error('uav:atc:ensure_atc_controller_on_path:RepoRootType', ...
        'Expected cfg.external_repo_root to be a text scalar.');
end
end
