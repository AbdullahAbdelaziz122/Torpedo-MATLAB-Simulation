%% navigation_lib.m  —  Navigation Module Function Library
%
% PURPOSE:
%   All navigation processing functions for Module M6.
%   Phase 4 implements a pass-through observer: eta_hat = eta_true,
%   nu_hat = nu_true. Noise injection and EKF are added in a later phase
%   by modifying ONLY this file and navigation_sfcn.m — no other module
%   changes.
%
% ARCHITECTURE CONTRACT:
%   Every module that needs vehicle state (Control M3, Guidance M2) must
%   read from NavBus outputs. They must never tap the Dynamics state
%   directly. This file is the single point where that contract is enforced.
%
% FUNCTIONS:
%   nav_passthrough    — Phase 4: perfect state pass-through (no noise)
%   extract_nu         — unpack nu = x(1:6) from raw state vector
%   extract_eta        — unpack eta = x(7:12) from raw state vector
%   wrap_angle         — wrap angle to [-pi, pi]
%   wrap_eta           — wrap all three Euler angles in eta
%   compute_speed      — scalar speed |nu| from body velocities
%   add_sensor_noise   — inject configurable Gaussian noise (Phase 4+)
%   nav_output         — assemble NavBus-compatible struct from estimates
%
% STATE VECTOR CONVENTION (remus100.m ordering):
%   x(1:6)  = nu  = [u, v, w, p, q, r]
%   x(7:12) = eta = [x_n, y_e, z_d, phi, theta, psi]
%
% AUTHOR: AUV Simulation Project — Phase 4

% =========================================================================
function nav = nav_passthrough(x_true, auv)
%% nav_passthrough  —  Perfect state observer (Phase 4)
%
% Returns navigation estimates identical to true state.
% Noise parameters in auv.nav are read but set to zero in Phase 4.
% To add noise later: set auv.nav.sigma_* > 0 in auv_params.m.
%
% INPUTS:
%   x_true  [12×1]  true state from Dynamics
%   auv             parameter struct
%
% OUTPUT:
%   nav     struct  with fields: nu_hat, eta_hat, speed
%                   (matches NavBus element names)

nu_true  = extract_nu(x_true);
eta_true = extract_eta(x_true);

% Inject noise if configured (sigma = 0 in Phase 4 → no effect)
nu_hat  = nu_true  + add_sensor_noise([auv.nav.sigma_vel;   auv.nav.sigma_vel; ...
                                        auv.nav.sigma_vel;   auv.nav.sigma_rate; ...
                                        auv.nav.sigma_rate;  auv.nav.sigma_rate]);

eta_hat = eta_true + add_sensor_noise([auv.nav.sigma_pos;   auv.nav.sigma_pos; ...
                                        auv.nav.sigma_pos;   auv.nav.sigma_angle; ...
                                        auv.nav.sigma_angle; auv.nav.sigma_angle]);

% Wrap Euler angles to [-pi, pi] after noise addition
eta_hat = wrap_eta(eta_hat);

% Assemble output struct
nav.nu_hat  = nu_hat;
nav.eta_hat = eta_hat;
nav.speed   = compute_speed(nu_hat);

end

% =========================================================================
function nu = extract_nu(x)
%% extract_nu  —  Pull body-frame velocities from state vector
% Returns [u; v; w; p; q; r] — first 6 elements of x.

nu = x(1:6);

end

% =========================================================================
function eta = extract_eta(x)
%% extract_eta  —  Pull NED position and Euler angles from state vector
% Returns [x_n; y_e; z_d; phi; theta; psi] — elements 7–12 of x.

eta = x(7:12);

end

% =========================================================================
function angle_out = wrap_angle(angle_in)
%% wrap_angle  —  Wrap a scalar angle to [-pi, pi]
%
% Uses atan2(sin, cos) — numerically stable, no discontinuity at ±pi.
% Critical for heading (psi) and roll (phi): without wrapping, accumulated
% angle drift causes guidance and control errors as angles cross ±pi.

angle_out = atan2(sin(angle_in), cos(angle_in));

end

% =========================================================================
function eta_wrapped = wrap_eta(eta)
%% wrap_eta  —  Wrap all three Euler angles in an eta vector
%
% Applies wrap_angle to phi (index 4), theta (index 5), psi (index 6).
% x_n, y_e, z_d (indices 1–3) are positions — not wrapped.

eta_wrapped      = eta;
eta_wrapped(4)   = wrap_angle(eta(4));   % phi
eta_wrapped(5)   = wrap_angle(eta(5));   % theta
eta_wrapped(6)   = wrap_angle(eta(6));   % psi

end

% =========================================================================
function speed = compute_speed(nu)
%% compute_speed  —  Scalar vehicle speed |nu| (m/s)
%
% Euclidean norm of body-frame linear velocities [u, v, w].
% Angular rates [p, q, r] are excluded — this is translational speed.

speed = sqrt(nu(1)^2 + nu(2)^2 + nu(3)^2);

end

% =========================================================================
function noise = add_sensor_noise(sigma_vec)
%% add_sensor_noise  —  Gaussian noise injection
%
% INPUTS:
%   sigma_vec  [6×1]  standard deviations per state element
%                     Set to zero for no noise (Phase 4 default)
%
% OUTPUT:
%   noise      [6×1]  noise sample (zero when all sigma = 0)
%
% When sigma = 0 for all elements, this function returns exact zeros
% with no randn calls — deterministic behaviour for Phase 4.

if all(sigma_vec == 0)
    noise = zeros(6, 1);
else
    noise = sigma_vec .* randn(6, 1);
end

end

% =========================================================================
function nav_validate(x, auv)
%% nav_validate  —  Sanity check on state vector (call during testing)
%
% Checks for NaN, Inf, and physically unreasonable values.
% Does not error — prints warnings only, so simulation continues.

if any(isnan(x))
    warning('navigation_lib: NaN detected in state vector.');
end
if any(isinf(x))
    warning('navigation_lib: Inf detected in state vector.');
end

u = x(1);
if abs(u) > 5.0
    warning('navigation_lib: surge velocity u=%.2f m/s exceeds physical limit.', u);
end

z = x(9);
if z > 500
    warning('navigation_lib: depth z=%.1f m seems unreasonably large.', z);
end

end
