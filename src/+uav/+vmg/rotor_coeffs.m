function rotor = rotor_coeffs(params)
%ROTOR_COEFFS Return rotor coefficients from a single parameter source.
% Description:
%   Reads thrust and torque coefficients from params.rotor when present and
%   falls back to legacy top-level fields for Stage-1 compatibility.
%
% Inputs:
%   params - parameter struct with params.rotor.* or legacy coefficient fields
%
% Outputs:
%   rotor - struct with kT_N_per_radps2 and kQ_Nm_per_radps2
%
% Units:
%   kT [N/(rad/s)^2], kQ [N*m/(rad/s)^2]
%
% Assumptions:
%   Nested params.rotor fields are the preferred source of truth.

if isfield(params, 'rotor') ...
        && isfield(params.rotor, 'kT_N_per_radps2') ...
        && isfield(params.rotor, 'kQ_Nm_per_radps2')
    rotor = params.rotor;
    return;
end

rotor = struct( ...
    'kT_N_per_radps2', params.kT_N_per_radps2, ...
    'kQ_Nm_per_radps2', params.kQ_Nm_per_radps2);
end
