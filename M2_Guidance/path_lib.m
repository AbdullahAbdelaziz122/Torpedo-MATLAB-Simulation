%% path_lib.m  —  Path Generator Library  (Phase 6)
%
% PURPOSE:
%   Generates reference paths and provides a query interface used by
%   the Guidance module. Separating path generation from the guidance
%   law means you can change the mission path without touching LOS.
%
% SUPPORTED PATHS:
%   path_helix      — helical spiral (default test path)
%   path_waypoints  — straight-line segments between waypoints
%   path_query      — get virtual point + velocity at time t
%   path_closest_t  — find t giving the closest point to vehicle position
%
% USAGE:
%   path = path_helix(auv);          % build path struct
%   [pt, vel] = path_query(path, t); % query at parameter t
%
% PATH STRUCT FIELDS:
%   path.type       — string identifier ('helix', 'waypoints')
%   path.t_vec      — parameter vector (s)
%   path.pts        — [3×N] position array [x_n; y_e; z_d]
%   path.vel        — [3×N] velocity array [dx/dt; dy/dt; dz/dt]
%   path.len        — total arc length (m)
%   path.T_total    — total time span (s)
%   path.U_nom      — nominal path speed (m/s)
%
% AUTHOR: AUV Simulation Project — Phase 6

% =========================================================================
function path = path_helix(auv)
%% path_helix  —  Helical spiral test path
%
% Matches the Ariza SimulatorFileRemusPID helix exactly:
%   Radius = 60 m, angular rate = 0.02618 rad/s_ramp
%   Ramp slope m = 0.6 (controls how fast along the helix)
%   Depth rate: from 2m to 12m over 200 s of ramp time
%
% INPUTS:
%   auv     parameter struct (uses auv.sim.T_end for duration)

T_end  = auv.sim.T_end;
dt     = auv.sim.Ts;
time   = 0 : dt : T_end;

% Path slope (controls parametric speed along helix)
m_ramp    = 0.6;
Yramp     = m_ramp * time;

% Position
X = 60 * cos(0.02618 * Yramp);
Y = 60 * sin(0.02618 * Yramp);
Z = 2 + (2 * Yramp / 200);        % depth: starts 2m, increases slowly

% Velocity (analytic derivatives)
dXdt = -60 * 0.02618 * m_ramp * sin(0.02618 * Yramp);
dYdt =  60 * 0.02618 * m_ramp * cos(0.02618 * Yramp);
dZdt =  2 * m_ramp / 200 * ones(size(time));

path.type    = 'helix';
path.t_vec   = time;
path.pts     = [X; Y; Z];
path.vel     = [dXdt; dYdt; dZdt];
path.T_total = T_end;
path.U_nom   = m_ramp;    % nominal path tangent speed magnitude (m/s)

% Arc length
ds         = sqrt(diff(X).^2 + diff(Y).^2 + diff(Z).^2);
path.len   = sum(ds);

% Starting heading for initial condition computation
path.psi0  = atan2(dYdt(1), dXdt(1));

end

% =========================================================================
function path = path_waypoints(wpts, U_nom, dt)
%% path_waypoints  —  Straight-line waypoint path
%
% INPUTS:
%   wpts    [3×M]  waypoints [x_n; y_e; z_d], M >= 2
%   U_nom   [1×1]  constant nominal speed along path (m/s)
%   dt      [1×1]  sample time (s)
%
% Generates a piecewise-linear path at constant speed.

if size(wpts,1) ~= 3 || size(wpts,2) < 2
    error('path_waypoints: wpts must be 3×M with M ≥ 2.');
end

% Build time-parameterised path at constant speed
segs   = diff(wpts, 1, 2);          % [3×(M-1)] segment vectors
dists  = sqrt(sum(segs.^2, 1));     % [1×(M-1)] segment lengths
T_segs = dists / U_nom;             % time per segment

T_total = sum(T_segs);
t_vec   = 0 : dt : T_total;
N       = numel(t_vec);

pts = zeros(3, N);
vel = zeros(3, N);

seg_start_t = [0, cumsum(T_segs)];

for k = 1:N
    t_k = t_vec(k);
    % Find which segment we are in
    seg = find(t_k >= seg_start_t(1:end-1) & t_k < seg_start_t(2:end), 1, 'last');
    if isempty(seg),  seg = size(wpts,2) - 1;  end  % clamp at end

    frac       = (t_k - seg_start_t(seg)) / T_segs(seg);
    frac       = min(max(frac, 0), 1);
    pts(:,k)   = wpts(:,seg) + frac * segs(:,seg);
    unit_tang  = segs(:,seg) / dists(seg);
    vel(:,k)   = unit_tang * U_nom;
end

path.type    = 'waypoints';
path.t_vec   = t_vec;
path.pts     = pts;
path.vel     = vel;
path.T_total = T_total;
path.U_nom   = U_nom;
path.len     = sum(dists);
path.psi0    = atan2(segs(2,1), segs(1,1));

end

% =========================================================================
function [pt, vel] = path_query(path, t)
%% path_query  —  Get path position and velocity at parameter t
%
% INPUTS:
%   path    path struct from path_helix or path_waypoints
%   t       query time (s), clamped to [0, T_total]
%
% OUTPUTS:
%   pt      [3×1]  position [x_n; y_e; z_d] at t
%   vel     [3×1]  velocity [dx/dt; dy/dt; dz/dt] at t

t   = max(0, min(t, path.T_total));
idx = max(1, min(round(t / (path.t_vec(2) - path.t_vec(1))) + 1, ...
              size(path.pts, 2)));

pt  = path.pts(:, idx);
vel = path.vel(:, idx);

end

% =========================================================================
function t_closest = path_closest_t(path, pos)
%% path_closest_t  —  Find path parameter t closest to vehicle position
%
% Brute-force nearest-neighbour search. Fast enough for paths of
% length < 10000 points. For longer paths, use a KD-tree instead.
%
% INPUTS:
%   path    path struct
%   pos     [3×1]  current vehicle position [x_n; y_e; z_d]
%
% OUTPUT:
%   t_closest  parameter t of closest path point

dists     = sum((path.pts - pos(:)).^2, 1);
[~, idx]  = min(dists);
t_closest = path.t_vec(idx);

end
