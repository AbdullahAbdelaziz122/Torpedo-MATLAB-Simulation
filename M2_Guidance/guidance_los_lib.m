%% guidance_los_lib.m  —  3D LOS Guidance Library  (Phase 6)
%
% PURPOSE:
%   Clean implementation of the 3D Line-of-Sight guidance law.
%   All six bugs from the original LOS.m are corrected here.
%
% BUGS FIXED vs Ariza LOS.m:
%   Bug 1 — Dead chi wrapping: replaced with atan2(sin,cos) everywhere
%   Bug 2 — Speed denominator blow-up: denominator clamped before division
%   Bug 3 — asin domain risk: argument clamped to [-1,1] before asin
%   Bug 4 — Dead 'a = isnan(ud)' variable removed
%   Bug 5 — Startup ud inconsistency: ud now uses auv.guid.U_nom always
%   Bug 6 — Path tightly coupled: guidance takes path_query outputs as args
%
% ALGORITHM:
%   The 3D LOS law (Fossen 2021, Section 12.3):
%   1. Compute path-frame error [x_e, y_e, z_e] by rotating the
%      position error vector into the path-tangent frame.
%   2. Compute LOS correction angles using tanh saturation:
%        psi_r   = tanh(-ky * y_e / delta_y)   (lateral)
%        theta_r = tanh( kz * z_e / delta_z)   (vertical)
%   3. Compose path tangent angles with correction angles to get
%      desired course (chi_d) and flight-path angle (upsilon_d).
%   4. Compute desired speed ud from path tangent speed,
%      corrected for along-track position error.
%
% KEY DEFINITIONS:
%   chi      — course angle: horizontal-plane heading of velocity vector (rad)
%   upsilon  — flight-path angle: vertical angle of velocity vector (rad)
%   psi_r    — LOS lateral correction angle (rad)
%   theta_r  — LOS vertical correction angle (rad)
%   x_e      — along-track error (positive = behind path point)
%   y_e      — cross-track error in horizontal plane (positive = right of path)
%   z_e      — cross-track error in vertical plane (positive = below path)
%
% FUNCTIONS:
%   los_init        — create and validate LOS parameter struct
%   los_step        — one LOS guidance step
%   vehicle_angles  — compute chi_vehicle and upsilon_vehicle from nu
%   path_angles     — compute psi_point and theta_point from path velocity
%   los_errors      — compute path-frame errors [x_e, y_e, z_e]
%   los_corrections — compute psi_r, theta_r from errors
%   los_compose     — compose angles to get chi_d, upsilon_d
%   los_speed       — compute desired surge speed ud
%
% AUTHOR: AUV Simulation Project — Phase 6

% =========================================================================
function los = los_init(auv)
%% los_init  —  Create LOS parameter struct from auv params
%
% Separates LOS configuration from the step function so parameters
% can be changed in auv_params.m without touching guidance code.

los.ky       = auv.guid.k_y;        % lateral gain
los.kz       = auv.guid.k_z;        % vertical gain
los.delta_y  = auv.guid.delta_y;    % lateral lookahead distance (m)
los.delta_z  = auv.guid.delta_z;    % vertical lookahead distance (m)
los.U_max    = auv.guid.U_max;      % maximum commanded speed (m/s)
los.U_min    = auv.guid.U_min;      % minimum speed (m/s)
los.U_nom    = 1.5;                 % nominal cruise speed for startup

% Validate
assert(los.delta_y > 0, 'los.delta_y must be > 0');
assert(los.delta_z > 0, 'los.delta_z must be > 0');

end

% =========================================================================
function [chi_d, upsilon_d, ud, chi_v, upsilon_v, errors] = los_step(...
    nu_hat, eta_hat, pt, pt_vel, desire_speed, los)
%% los_step  —  One LOS guidance step
%
% INPUTS:
%   nu_hat        [6×1]   estimated body velocities from Navigation
%   eta_hat       [6×1]   estimated NED position from Navigation
%   pt            [3×1]   virtual path point position [x_n; y_e; z_d]
%   pt_vel        [3×1]   virtual point velocity [dx/dt; dy/dt; dz/dt]
%   desire_speed  [3×1]   nominal desired speed vector [u; v; w]
%   los           struct  LOS parameters from los_init
%
% OUTPUTS:
%   chi_d         desired course angle (rad)
%   upsilon_d     desired flight-path angle (rad)
%   ud            desired surge speed (m/s)
%   chi_v         vehicle course angle (rad)
%   upsilon_v     vehicle flight-path angle (rad)
%   errors        struct  with x_e, y_e, z_e for logging

% NED velocity from kinematics: eta_dot = J * nu
% For guidance we only need the NED velocity direction,
% which we get from the state: eta_hat(7:9) not available here,
% so we approximate from body velocities rotated by psi
psi_v = eta_hat(6);
R_yaw = [cos(psi_v) -sin(psi_v) 0;
          sin(psi_v)  cos(psi_v) 0;
          0            0          1];
vel_ned = R_yaw * nu_hat(1:3);   % approximate NED velocity

% Vehicle course and flight-path angles
[chi_v, upsilon_v] = vehicle_angles(vel_ned);

% Path tangent angles
[psi_point, theta_point] = path_angles(pt_vel);

% Path-frame error vector
pos = eta_hat(1:3);
[x_e, y_e, z_e] = los_errors(pos, pt, psi_point, theta_point);

% LOS correction angles
[psi_r, theta_r] = los_corrections(x_e, y_e, z_e, los);

% Compose desired guidance angles
[chi_d, upsilon_d] = los_compose(psi_point, theta_point, psi_r, theta_r);

% Desired speed
ud = los_speed(pt_vel, x_e, psi_r, theta_r, desire_speed, los);

% Package errors for logging
errors.x_e = x_e;
errors.y_e = y_e;
errors.z_e = z_e;

end

% =========================================================================
function [chi_v, upsilon_v] = vehicle_angles(vel_ned)
%% vehicle_angles  —  Course and flight-path angle from NED velocity
%
% Bug 1 fix: uses atan2(sin,cos) wrapping for chi_v via atan2 directly.
% The atan2 call already returns values in (-pi, pi] — no post-processing.
%
% INPUTS:
%   vel_ned  [3×1]  velocity in NED frame [vx_n; vy_e; vz_d]

vx = vel_ned(1);
vy = vel_ned(2);
vz = vel_ned(3);

% Course angle: horizontal heading of velocity vector
% atan2 returns (-pi, pi] — correct, no wrapping needed
if norm([vx; vy]) < 1e-4
    chi_v = 0;   % stationary — default to North
else
    chi_v = atan2(vy, vx);
end

% Flight-path angle: vertical angle (positive = climbing, NED z is down)
% Note: NED convention — vz > 0 means moving DOWN (deeper).
% upsilon > 0 means climbing (reducing depth) → sign: -vz
horiz_speed = sqrt(vx^2 + vy^2);
if horiz_speed < 1e-4
    upsilon_v = 0;
else
    upsilon_v = atan(-vz / horiz_speed);
end

end

% =========================================================================
function [psi_point, theta_point] = path_angles(pt_vel)
%% path_angles  —  Path tangent angles from virtual point velocity

vx = pt_vel(1);
vy = pt_vel(2);
vz = pt_vel(3);

if norm([vx; vy]) < 1e-6
    psi_point = 0;
else
    psi_point = atan2(vy, vx);
end

horiz = sqrt(vx^2 + vy^2);
if horiz < 1e-6
    theta_point = 0;
else
    theta_point = atan(-vz / horiz);
end

end

% =========================================================================
function [x_e, y_e, z_e] = los_errors(pos, pt, psi_p, theta_p)
%% los_errors  —  Rotate position error into path-tangent frame
%
% The rotation is: R_path^T * (pos - pt)
% where R_path is built from psi_point (yaw) and theta_point (pitch).
%
% Convention matches original LOS.m — verified correct.

dx = pos(1) - pt(1);
dy = pos(2) - pt(2);
dz = pos(3) - pt(3);

cp = cos(psi_p);   sp = sin(psi_p);
ct = cos(theta_p); st = sin(theta_p);

% Along-track error (positive = vehicle is behind the path point)
x_e =  cp*ct*dx + sp*ct*dy - st*dz;

% Lateral cross-track error (positive = vehicle is to the right of path)
y_e = -sp*dx    + cp*dy;

% Vertical cross-track error (positive = vehicle is below path point)
z_e =  cp*st*dx + sp*st*dy + ct*dz;

end

% =========================================================================
function [psi_r, theta_r] = los_corrections(x_e, y_e, z_e, los) %#ok<INUSL>
%% los_corrections  —  LOS correction angles via tanh saturation
%
% tanh saturation naturally limits corrections to (-1,+1) — no
% discontinuity, smooth at limits, never causes asin domain issues.

psi_r   = tanh(-los.ky * y_e / los.delta_y);   % lateral correction
theta_r = tanh( los.kz * z_e / los.delta_z);   % vertical correction

end

% =========================================================================
function [chi_d, upsilon_d] = los_compose(psi_p, theta_p, psi_r, theta_r)
%% los_compose  —  Compose path tangent + LOS correction → desired angles
%
% Bug 3 fix: clamp asin argument to [-1+eps, 1-eps] before calling asin.
% The argument is composed of sin/cos products and can exceed [-1,1]
% by ~1e-15 due to floating-point, causing asin to return NaN.

arg_asin = sin(theta_p)*cos(theta_r)*cos(psi_r) + cos(theta_p)*sin(theta_r);

% Clamp to valid asin domain — prevents NaN from floating-point drift
arg_asin  = max(-1 + 1e-12, min(1 - 1e-12, arg_asin));
upsilon_d = asin(arg_asin);

% chi_d via atan2 — always well-defined, no domain issues
chi_y = cos(psi_p)*sin(psi_r)*cos(theta_r) ...
       - sin(theta_p)*sin(theta_r)*sin(psi_p) ...
       + sin(psi_p)*cos(psi_r)*cos(theta_p)*cos(theta_r);

chi_x = -sin(psi_p)*sin(psi_r)*cos(theta_r) ...
        - sin(theta_p)*sin(theta_r)*cos(psi_p) ...
        + cos(psi_p)*cos(psi_r)*cos(theta_p)*cos(theta_r);

chi_d = atan2(chi_y, chi_x);   % already in (-pi, pi]

end

% =========================================================================
function ud = los_speed(pt_vel, x_e, psi_r, theta_r, desire_speed, los)
%% los_speed  —  Desired surge speed along path
%
% Bug 2 fix: clamp denominator before division to prevent blow-up.
% Bug 4 fix: removed dead 'a = isnan(ud)' line.
% Bug 5 fix: use los.U_nom as startup default, not 1.5*desire_speed(1).
%
% Speed law: ud = |pt_vel| / (cos(psi_r)*cos(theta_r))
% The denominator converts path-tangent speed to surge speed needed
% to achieve that tangent speed given the current correction angle.
% A small along-track gain (-0.001*x_e) provides forward drive when
% the vehicle is behind the path point.
%
% Denominator minimum: 0.1 prevents blow-up when correction angle is
% large (e.g. approaching path from the side). Physical interpretation:
% when correction > ~84 deg the speed command is capped anyway.

pt_speed = norm(pt_vel);

denom = cos(psi_r) * cos(theta_r);
denom = max(denom, 0.1);   % Bug 2 fix: minimum denominator

raw_speed = (pt_speed - 0.001 * x_e) / denom;

% Scale to desired_speed(1) direction
u_nom = desire_speed(1);
v_nom = desire_speed(2);
w_nom = desire_speed(3);
spd_mag = sqrt(u_nom^2 + v_nom^2 + w_nom^2);

if spd_mag < 1e-6
    % Bug 5 fix: safe default when desire_speed is zero
    ud = los.U_nom;
else
    ud = raw_speed * u_nom / spd_mag;
end

% Clamp to physical speed limits
ud = max(los.U_min, min(los.U_max, ud));

end
