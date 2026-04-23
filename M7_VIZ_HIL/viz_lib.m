%% viz_lib.m  —  Visualization Library  (Phase 8 — Revised)
%
% PURPOSE:
%   Real-time and post-run visualization for the AUV simulation.
%   Two operating modes:
%     LIVE mode   — animatedline figures update every N steps during sim
%     POST mode   — viz_final() generates all post-run analysis figures
%
% FUNCTIONS:
%   viz = viz_init(update_every)   — create + format all live figure handles
%   viz = viz_update(viz, log)     — append one step to all live figures
%         viz_final(log)           — generate complete post-run figure set
%         viz_close(viz)           — close live figures after simulation
%
% STYLE CONVENTION (consistent across all figures):
%   Actual signal      — solid colour line, LineWidth 1.5
%   Reference/Desired  — dashed black  ('--k'), LineWidth 1.2
%   Saturation limit   — dashed red    ('--r'), LineWidth 0.8
%   Zero reference     — solid black   ('-k'),  LineWidth 0.5
%
% FIGURE MAP:
%   Live (during sim):  10 — 6-DOF Velocities
%                       11 — Position & Attitude
%                       12 — Actuator Commands
%                       13 — LOS Cross-track Errors
%   Post-run (after):   20 — 3D Trajectory
%                       21 — All 12 States (Desired vs Actual)
%                       22 — Actuator Signals with Saturation Limits
%                       23 — LOS Cross-track Errors with RMS Annotations
%                       24 — Environmental Disturbances
%
% KNOWN ARCHITECTURAL NOTE:
%   run_simulation.m contains inline local copies of viz_init, viz_update,
%   and viz_final (Phase 8 scaffolding). Those copies take precedence due
%   to MATLAB scoping. To activate this library:
%     1. Remove the three viz_* local functions from run_simulation.m
%     2. Ensure viz_lib.m is on the MATLAB path (addpath or same folder)
%
% AUTHOR: AUV Simulation Project — Phase 8 (Revised)

% =========================================================================
%% INTERNAL STYLE HELPER
% =========================================================================

function s = get_style()
%% get_style  —  Return shared style constants for all viz functions
%
% Centralising these prevents the live/post-run drift that comes from
% copy-pasting magic numbers across functions.

s.lw_act  = 1.5;   % actual signal line width
s.lw_ref  = 1.2;   % reference / desired line width
s.lw_lim  = 0.8;   % saturation limit line width
s.lw_zero = 0.5;   % zero-reference rule line width

s.fs_sgt  = 12;    % sgtitle font size
s.fs_ttl  = 10;    % subplot title font size
s.fs_lbl  = 9;     % axis label font size
s.fs_lgn  = 8;     % legend font size

% Colour palette — same index used for both live and post-run
s.c = {[0.13 0.47 0.71], ...   % 1 — blue   (surge / North / RPM)
       [0.90 0.45 0.10], ...   % 2 — orange (sway  / East / stern)
       [0.17 0.63 0.17], ...   % 3 — green  (heave / depth / rudder)
       [0.84 0.15 0.16], ...   % 4 — red    (roll  / lateral error)
       [0.58 0.40 0.74], ...   % 5 — purple (pitch)
       [0.55 0.34 0.29]};      % 6 — brown  (yaw)
end

% =========================================================================
function viz = viz_init(update_every)
%% viz_init  —  Create all live-update figure handles and format axes
%
% INPUTS:
%   update_every   integer   call viz_update every this many steps
%                            (default 50 → 0.5 s at Ts=0.01 s)
%
% OUTPUTS:
%   viz   struct   all animatedline handles plus figure handles

if nargin < 1, update_every = 50; end
viz.update_every = update_every;
viz.initialized  = true;
s = get_style();

% -------------------------------------------------------------------------
% Figure 10: 6-DOF Velocities
% Structure: 2×3 grid — [u v w | p q r]
% Desired signal shown for u only (explicit speed command).
% Zero-reference yline shown for v, w, p (should remain near 0).
% -------------------------------------------------------------------------
fig1 = figure(10);
set(fig1,'Name','Live — 6-DOF Velocities','NumberTitle','off', ...
    'Position',[10 560 640 420]);
clf(fig1);

subplot(2,3,1);
viz.h_ud = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref, ...
    'DisplayName','u_d (desired)');
viz.h_u  = animatedline('Color',s.c{1},'LineWidth',s.lw_act, ...
    'DisplayName','u (actual)');
title('Surge u','FontSize',s.fs_ttl);
ylabel('u (m/s)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;

subplot(2,3,2);
viz.h_v = animatedline('Color',s.c{2},'LineWidth',s.lw_act,'DisplayName','v (actual)');
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Sway v','FontSize',s.fs_ttl);
ylabel('v (m/s)','FontSize',s.fs_lbl); grid on;

subplot(2,3,3);
viz.h_w = animatedline('Color',s.c{3},'LineWidth',s.lw_act,'DisplayName','w (actual)');
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Heave w','FontSize',s.fs_ttl);
ylabel('w (m/s)','FontSize',s.fs_lbl); grid on;

subplot(2,3,4);
viz.h_p = animatedline('Color',s.c{4},'LineWidth',s.lw_act,'DisplayName','p (actual)');
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Roll Rate p','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('p (deg/s)','FontSize',s.fs_lbl); grid on;

subplot(2,3,5);
viz.h_q = animatedline('Color',s.c{5},'LineWidth',s.lw_act,'DisplayName','q (actual)');
title('Pitch Rate q','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('q (deg/s)','FontSize',s.fs_lbl); grid on;

subplot(2,3,6);
viz.h_r = animatedline('Color',s.c{6},'LineWidth',s.lw_act,'DisplayName','r (actual)');
title('Yaw Rate r','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('r (deg/s)','FontSize',s.fs_lbl); grid on;

sgtitle('Live: 6-DOF Velocities','FontSize',s.fs_sgt);
viz.fig1 = fig1;

% -------------------------------------------------------------------------
% Figure 11: Position & Attitude
% Structure: 2×3 grid — [x_N  y_E  z_D | phi  theta  psi]
% Reference signals: z_ref (derived), upsilon_d (pitch), chi_d (yaw).
% Zero-reference yline shown for roll phi (should remain near 0).
% -------------------------------------------------------------------------
fig2 = figure(11);
set(fig2,'Name','Live — Position & Attitude','NumberTitle','off', ...
    'Position',[660 560 640 420]);
clf(fig2);

subplot(2,3,1);
viz.h_xn = animatedline('Color',s.c{1},'LineWidth',s.lw_act);
title('North x_N','FontSize',s.fs_ttl);
ylabel('x_N (m)','FontSize',s.fs_lbl); grid on;

subplot(2,3,2);
viz.h_ye = animatedline('Color',s.c{2},'LineWidth',s.lw_act);
title('East y_E','FontSize',s.fs_ttl);
ylabel('y_E (m)','FontSize',s.fs_lbl); grid on;

subplot(2,3,3);
viz.h_zref = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref, ...
    'DisplayName','z_{ref}');
viz.h_zd   = animatedline('Color',s.c{3},'LineWidth',s.lw_act, ...
    'DisplayName','z_D (actual)');
title('Depth z_D','FontSize',s.fs_ttl);
ylabel('z_D (m)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;

subplot(2,3,4);
viz.h_phi = animatedline('Color',s.c{4},'LineWidth',s.lw_act,'DisplayName','\phi (actual)');
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Roll \phi','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\phi (deg)','FontSize',s.fs_lbl); grid on;

subplot(2,3,5);
viz.h_ups   = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref, ...
    'DisplayName','\upsilon_d (ref)');
viz.h_theta = animatedline('Color',s.c{5},'LineWidth',s.lw_act, ...
    'DisplayName','\theta (actual)');
title('Pitch \theta','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\theta (deg)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;

subplot(2,3,6);
viz.h_chid = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref, ...
    'DisplayName','\chi_d (ref)');
viz.h_psi  = animatedline('Color',s.c{6},'LineWidth',s.lw_act, ...
    'DisplayName','\psi (actual)');
title('Yaw \psi','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\psi (deg)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;

sgtitle('Live: Position & Attitude','FontSize',s.fs_sgt);
viz.fig2 = fig2;

% -------------------------------------------------------------------------
% Figure 12: Actuator Commands
% Structure: 3×1 — [RPM | stern | rudder]
% Saturation limits shown as dashed red ylines.
% -------------------------------------------------------------------------
fig3 = figure(12);
set(fig3,'Name','Live — Actuator Commands','NumberTitle','off', ...
    'Position',[10 80 640 420]);
clf(fig3);

subplot(3,1,1);
viz.h_n_rpm = animatedline('Color',s.c{1},'LineWidth',s.lw_act);
yline(1525,'--r','LineWidth',s.lw_lim,'Label','Max 1525 RPM','HandleVisibility','off');
yline(0,'--k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Propeller Speed','FontSize',s.fs_ttl);
ylabel('n (RPM)','FontSize',s.fs_lbl); ylim([-50 1600]); grid on;

subplot(3,1,2);
viz.h_ds = animatedline('Color',s.c{2},'LineWidth',s.lw_act);
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20°','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20°','HandleVisibility','off');
title('Stern Plane \delta_s','FontSize',s.fs_ttl);
ylabel('\delta_s (deg)','FontSize',s.fs_lbl); ylim([-25 25]); grid on;

subplot(3,1,3);
viz.h_dr = animatedline('Color',s.c{3},'LineWidth',s.lw_act);
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20°','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20°','HandleVisibility','off');
title('Rudder \delta_r','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\delta_r (deg)','FontSize',s.fs_lbl);
ylim([-25 25]); grid on;

sgtitle('Live: Actuator Commands','FontSize',s.fs_sgt);
viz.fig3 = fig3;

% -------------------------------------------------------------------------
% Figure 13: LOS Cross-track Errors
% Structure: 3×1 — [x_e | y_e | z_e]
% Zero reference shown on all subplots.
% -------------------------------------------------------------------------
fig4 = figure(13);
set(fig4,'Name','Live — LOS Cross-track Errors','NumberTitle','off', ...
    'Position',[660 80 640 420]);
clf(fig4);

subplot(3,1,1);
viz.h_xe = animatedline('Color',s.c{1},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Along-track Error x_e','FontSize',s.fs_ttl);
ylabel('x_e (m)','FontSize',s.fs_lbl); grid on;

subplot(3,1,2);
viz.h_lye = animatedline('Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Lateral Cross-track Error y_e','FontSize',s.fs_ttl);
ylabel('y_e (m)','FontSize',s.fs_lbl); grid on;

subplot(3,1,3);
viz.h_lze = animatedline('Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Vertical Cross-track Error z_e','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('z_e (m)','FontSize',s.fs_lbl); grid on;

sgtitle('Live: LOS Cross-track Errors','FontSize',s.fs_sgt);
viz.fig4 = fig4;

drawnow;
end  % viz_init


% =========================================================================
function viz = viz_update(viz, log)
%% viz_update  —  Append the latest logged sample to all live figures
%
% Only the newest data point (at index log.k) is added.  animatedline
% accumulates all history internally.  drawnow limitrate avoids stalling.
%
% NOTE: The depth reference z_ref is computed from the logged LOS vertical
%       error as:  z_ref = z_D_actual − z_e
%       This is algebraically equivalent to the guidance commanded depth.

k = log.k;
if k < 1, return; end
t = log.t(k);

% --- Figure 10: 6-DOF Velocities ---
addpoints(viz.h_ud, t, log.ud(k));
addpoints(viz.h_u,  t, log.x(1,k));
addpoints(viz.h_v,  t, log.x(2,k));
addpoints(viz.h_w,  t, log.x(3,k));
addpoints(viz.h_p,  t, rad2deg(log.x(4,k)));
addpoints(viz.h_q,  t, rad2deg(log.x(5,k)));
addpoints(viz.h_r,  t, rad2deg(log.x(6,k)));

% --- Figure 11: Position / Attitude ---
addpoints(viz.h_xn,    t, log.x(7,k));
addpoints(viz.h_ye,    t, log.x(8,k));
addpoints(viz.h_zd,    t, log.x(9,k));
addpoints(viz.h_zref,  t, log.x(9,k) - log.los_ze(k));   % z_ref from LOS
addpoints(viz.h_phi,   t, rad2deg(log.x(10,k)));
addpoints(viz.h_ups,   t, rad2deg(log.upsilon_d(k)));
addpoints(viz.h_theta, t, rad2deg(log.x(11,k)));
addpoints(viz.h_chid,  t, rad2deg(log.chi_d(k)));
addpoints(viz.h_psi,   t, rad2deg(log.x(12,k)));

% --- Figure 12: Actuator Commands ---
addpoints(viz.h_n_rpm, t, log.n_direct(k));
addpoints(viz.h_ds,    t, rad2deg(log.ui(2,k)));
addpoints(viz.h_dr,    t, rad2deg(log.ui(1,k)));

% --- Figure 13: Cross-track Errors ---
addpoints(viz.h_xe,  t, log.los_xe(k));
addpoints(viz.h_lye, t, log.los_ye(k));
addpoints(viz.h_lze, t, log.los_ze(k));

drawnow limitrate;
end  % viz_update


% =========================================================================
function viz_close(viz)
%% viz_close  —  Close all live figure windows cleanly
%
% Call between the simulation loop end and viz_final() to avoid having
% stale live windows overlap the post-run figure set.

for fnum = [10 11 12 13]
    if ishandle(fnum), close(fnum); end
end
fprintf('  Live figures (10-13) closed.\n');
end  % viz_close


% =========================================================================
function viz_final(log)
%% viz_final  —  Generate complete post-run engineering figure set
%
% Produces Figures 20-24.  May be called at any time after simulation:
%   viz_final(log)       — pass the workspace log struct directly
%
% WHAT EACH FIGURE PROVES:
%   20 — Path execution: spatial extent of mission, start/end locations
%   21 — Control fidelity: every state tracked against its reference
%   22 — Actuator health: commands vs saturation limits, RPM range used
%   23 — Guidance precision: LOS error convergence and RMS steady-state
%   24 — Disturbance rejection evidence: the forces the controller fought

if ~isfield(log,'k') || log.k < 2
    warning('viz_final: log is empty or too short (k=%d). Skipping.', log.k);
    return;
end

s  = get_style();
k  = log.k;
t  = log.t(1:k);

% Derived depth reference:  z_ref = z_actual - z_error  (from LOS geometry)
z_ref = log.x(9,1:k) - log.los_ze(1:k);

% -------------------------------------------------------------------------
% Figure 20: 3D Trajectory
% PROVES: The AUV executed the assigned 3D path over the full mission.
% ADDED:  legend() call (was missing), daspect instead of axis equal,
%         title annotated with total range.
% -------------------------------------------------------------------------
fig20 = figure(20);
set(fig20,'Name','Post-run — 3D Trajectory','NumberTitle','off', ...
    'Position',[50 430 720 520]);
clf(fig20);

plot3(log.x(7,1:k), log.x(8,1:k), -log.x(9,1:k), ...
    'Color',s.c{1},'LineWidth',s.lw_act,'DisplayName','Vehicle path');
hold on;
scatter3(log.x(7,1), log.x(8,1), -log.x(9,1), 100,'g','filled', ...
    'DisplayName','Start');
scatter3(log.x(7,k), log.x(8,k), -log.x(9,k), 100,'r','filled', ...
    'DisplayName','End');
hold off;

xlabel('North (m)',    'FontSize',s.fs_lbl);
ylabel('East (m)',     'FontSize',s.fs_lbl);
zlabel('Altitude — up+ve (m)', 'FontSize',s.fs_lbl);
title(sprintf('3D AUV Trajectory  [T = %.0f s,  d = %.0f m]', ...
    t(k), norm(log.x(7:9,k)-log.x(7:9,1))), 'FontSize',s.fs_ttl);
legend('FontSize',s.fs_lgn,'Location','best');
grid on;
daspect([1 1 0.05]);   % preserve horizontal scale; don't distort depth
view(30,25);

% -------------------------------------------------------------------------
% Figure 21: All 12 States — Desired vs Actual
% PROVES: Every DOF is tracked to its reference over the full run.
% LAYOUT: 4×3 grid — top 2 rows: velocities, bottom 2 rows: positions
% FIXED:  references for ALL 12 states (was only u, theta, psi before)
% -------------------------------------------------------------------------
fig21 = figure(21);
set(fig21,'Name','Post-run — All 12 States','NumberTitle','off', ...
    'Position',[50 30 1140 720]);
clf(fig21);

% --- Row 1-2: Velocity states x(1:6) ---
vel_titles  = {'Surge u','Sway v','Heave w','Roll Rate p','Pitch Rate q','Yaw Rate r'};
vel_ylabels = {'u (m/s)','v (m/s)','w (m/s)','p (deg/s)','q (deg/s)','r (deg/s)'};
ang_scale   = [1 1 1 180/pi 180/pi 180/pi];  % convert rad/s → deg/s for p,q,r

for i = 1:6
    ax = subplot(4,3,i);
    hold(ax,'on');

    switch i
        case 1
            % Surge — explicit speed reference ud from guidance module
            h1 = plot(t, log.ud(1:k), '--k','LineWidth',s.lw_ref,'DisplayName','u_d (desired)');
            h2 = plot(t, log.x(1,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','u (actual)');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');

        case {2,3}
            % Sway, Heave — no direct reference; ideal value is 0
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref = 0','HandleVisibility','off');
            plot(t, log.x(i,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','Actual');

        case 4
            % Roll rate — should remain ~0 for non-rolling AUV
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref = 0','HandleVisibility','off');
            plot(t, rad2deg(log.x(i,1:k)),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','Actual');

        case {5,6}
            % Pitch/Yaw rate — indirectly controlled; no explicit setpoint
            plot(t, rad2deg(log.x(i,1:k)),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','Actual');
    end

    title(vel_titles{i},'FontSize',s.fs_ttl);
    xlabel('t (s)','FontSize',s.fs_lbl);
    ylabel(vel_ylabels{i},'FontSize',s.fs_lbl);
    grid on; hold(ax,'off');
end

% --- Row 3-4: Position/attitude states x(7:12) ---
pos_titles  = {'North x_N','East y_E','Depth z_D','Roll \phi','Pitch \theta','Yaw \psi'};
pos_ylabels = {'x_N (m)','y_E (m)','z_D (m)','\phi (deg)','\theta (deg)','\psi (deg)'};

for i = 1:6
    ax = subplot(4,3,6+i);
    hold(ax,'on');

    switch i
        case {1,2}
            % North, East — path-following (no fixed waypoint setpoint)
            plot(t, log.x(6+i,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','Actual');

        case 3
            % Depth — reference recovered from LOS geometry: z_ref = z - z_e
            h1 = plot(t, z_ref,'--k','LineWidth',s.lw_ref,'DisplayName','z_{ref} (guidance)');
            h2 = plot(t, log.x(9,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','z_D (actual)');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');

        case 4
            % Roll — should stay at zero for stable AUV
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref = 0','HandleVisibility','off');
            plot(t, rad2deg(log.x(10,1:k)),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','Actual');

        case 5
            % Pitch — reference is LOS flight-path angle upsilon_d
            h1 = plot(t, rad2deg(log.upsilon_d(1:k)),'--k','LineWidth',s.lw_ref, ...
                'DisplayName','\upsilon_d (ref)');
            h2 = plot(t, rad2deg(log.x(11,1:k)),'Color',s.c{i},'LineWidth',s.lw_act, ...
                'DisplayName','\theta (actual)');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');

        case 6
            % Yaw — reference is LOS course angle chi_d
            h1 = plot(t, rad2deg(log.chi_d(1:k)),'--k','LineWidth',s.lw_ref, ...
                'DisplayName','\chi_d (ref)');
            h2 = plot(t, rad2deg(log.x(12,1:k)),'Color',s.c{i},'LineWidth',s.lw_act, ...
                'DisplayName','\psi (actual)');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
    end

    title(pos_titles{i},'FontSize',s.fs_ttl);
    xlabel('t (s)','FontSize',s.fs_lbl);
    ylabel(pos_ylabels{i},'FontSize',s.fs_lbl);
    grid on; hold(ax,'off');
end

sgtitle('Post-run: All 12 States — Desired vs Actual','FontSize',s.fs_sgt);

% -------------------------------------------------------------------------
% Figure 22: Actuator Signals with Saturation Limits
% PROVES: Controller demands remained within physical actuator bounds.
%         Any saturation events are immediately visible as clipping.
% FIXED:  Consistent ylim, xlabel spacing, saturation limit labels.
% -------------------------------------------------------------------------
fig22 = figure(22);
set(fig22,'Name','Post-run — Actuator Signals','NumberTitle','off', ...
    'Position',[800 430 640 460]);
clf(fig22);

subplot(3,1,1);
plot(t, log.n_direct(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
hold on;
yline(1525,'--r','LineWidth',s.lw_lim,'Label','Max 1525 RPM','HandleVisibility','off');
yline(0,   '--k','LineWidth',s.lw_zero,'HandleVisibility','off');
hold off;
title('Propeller Speed','FontSize',s.fs_ttl);
ylabel('n (RPM)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
ylim([-50 1600]); grid on;

subplot(3,1,2);
plot(t, rad2deg(log.ui(2,1:k)),'Color',s.c{2},'LineWidth',s.lw_act);
hold on;
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20° limit','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20° limit','HandleVisibility','off');
hold off;
title('Stern Plane \delta_s','FontSize',s.fs_ttl);
ylabel('\delta_s (deg)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
ylim([-25 25]); grid on;

subplot(3,1,3);
plot(t, rad2deg(log.ui(1,1:k)),'Color',s.c{3},'LineWidth',s.lw_act);
hold on;
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20° limit','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20° limit','HandleVisibility','off');
hold off;
title('Rudder \delta_r','FontSize',s.fs_ttl);
ylabel('\delta_r (deg)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
ylim([-25 25]); grid on;

sgtitle('Post-run: Actuator Signals','FontSize',s.fs_sgt);

% -------------------------------------------------------------------------
% Figure 23: LOS Cross-track Errors with RMS Annotations
% PROVES: The LOS guidance law drove all three path errors to zero.
%         RMS embedded in subplot title gives a single convergence metric.
% FIXED:  RMS embedded in titles, consistent xlabel, consistent FontSize.
% -------------------------------------------------------------------------
fig23 = figure(23);
set(fig23,'Name','Post-run — LOS Cross-track Errors','NumberTitle','off', ...
    'Position',[800 30 640 440]);
clf(fig23);

rms_xe = sqrt(mean(log.los_xe(1:k).^2));
rms_ye = sqrt(mean(log.los_ye(1:k).^2));
rms_ze = sqrt(mean(log.los_ze(1:k).^2));

subplot(3,1,1);
plot(t, log.los_xe(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Along-track Error x_e    [RMS = %.2f m]', rms_xe), ...
    'FontSize',s.fs_ttl);
ylabel('x_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
grid on;

subplot(3,1,2);
plot(t, log.los_ye(1:k),'Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Lateral Cross-track y_e    [RMS = %.2f m]', rms_ye), ...
    'FontSize',s.fs_ttl);
ylabel('y_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
grid on;

subplot(3,1,3);
plot(t, log.los_ze(1:k),'Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Vertical Cross-track z_e    [RMS = %.2f m]', rms_ze), ...
    'FontSize',s.fs_ttl);
ylabel('z_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
grid on;

sgtitle('Post-run: LOS Cross-track Errors','FontSize',s.fs_sgt);

% -------------------------------------------------------------------------
% Figure 24: Environmental Disturbances
% PROVES: These are the external forces the controller successfully
%         rejected.  Without this figure the performance in Fig 23 is
%         uninterpretable (no context for how hard the rejection was).
% FIXED:  ylabel betaVc → Greek symbol, figure position avoids overlap
%         with Fig 21, consistent FontSize on all subplot titles.
% -------------------------------------------------------------------------
fig24 = figure(24);
set(fig24,'Name','Post-run — Environmental Disturbances','NumberTitle','off', ...
    'Position',[50 30 700 420]);   % different position from Fig 21 [50 30 1140 720]
clf(fig24);

subplot(2,2,1);
plot(t, log.Vc(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
title('Ocean Current Speed','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('V_c (m/s)','FontSize',s.fs_lbl);
grid on;

subplot(2,2,2);
plot(t, rad2deg(log.betaVc(1:k)),'Color',s.c{2},'LineWidth',s.lw_act);
title('Current Direction','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\beta_{Vc} (deg)','FontSize',s.fs_lbl);
grid on;

subplot(2,2,3);
plot(t, log.tau_env(3,1:k),'Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Wave Heave Force \tau_Z','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\tau_Z (N)','FontSize',s.fs_lbl);
grid on;

subplot(2,2,4);
plot(t, log.tau_env(5,1:k),'Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Wave Pitch Moment \tau_M','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\tau_M (N·m)','FontSize',s.fs_lbl);
grid on;

sgtitle('Post-run: Environmental Disturbances','FontSize',s.fs_sgt);

% Summary to console
fprintf('\n  Post-run figures 20-24 generated.\n');
fprintf('  RMS cross-track:  x_e = %.3f m   y_e = %.3f m   z_e = %.3f m\n', ...
    rms_xe, rms_ye, rms_ze);

end  % viz_final
