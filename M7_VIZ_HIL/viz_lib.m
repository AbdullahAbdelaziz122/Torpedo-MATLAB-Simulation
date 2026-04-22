%% viz_lib.m  —  Visualization Library  (Phase 8)
%
% PURPOSE:
%   Real-time and post-run visualization for the AUV simulation.
%   Two operating modes:
%     LIVE mode   — figures update every N steps during simulation
%     POST mode   — generate all figures from a completed log struct
%
% FUNCTIONS:
%   viz_init        — create figure handles and pre-allocate line objects
%   viz_update      — update live plots from log (call every Nplot steps)
%   viz_final       — generate all post-run figures from complete log
%   viz_3d_init     — initialise 3D animation figure
%   viz_3d_update   — update 3D vehicle position and trail
%   viz_export      — save all open figures to PNG files
%
% DESIGN:
%   All figures are created with explicit handles stored in a viz struct.
%   Updating uses set(handle,'XData',..,'YData',..) rather than re-plotting —
%   this is ~10x faster for live updates and keeps the axes stable.
%
% USAGE (live):
%   viz = viz_init();
%   for k = 1:N
%       ... simulate one step, update log ...
%       if mod(k, viz.update_every) == 0
%           viz = viz_update(viz, log);
%       end
%   end
%   viz_final(log);
%
% USAGE (post-run):
%   load('auv_log_20250422_120000.mat');
%   viz_final(log);
%
% AUTHOR: AUV Simulation Project — Phase 8

% =========================================================================
function viz = viz_init(update_every)
%% viz_init  —  Create all live-update figure handles
%
% INPUTS:
%   update_every   how many simulation steps between plot updates
%                  (default: 50 steps = 0.5s at Ts=0.01s)

if nargin < 1, update_every = 50; end

viz.update_every = update_every;
viz.initialized  = true;

% --- Figure 1: 6-DOF velocities ---
fig1 = figure(10);
set(fig1,'Name','Live — Velocities','NumberTitle','off', ...
    'Position',[10 560 640 420]);

subplot(2,3,1); viz.h_u  = animatedline('Color',[0.13 0.47 0.71],'LineWidth',1.2);
xlabel('t (s)'); ylabel('u (m/s)'); title('Surge'); grid on;

subplot(2,3,2); viz.h_v  = animatedline('Color',[0.90 0.45 0.10],'LineWidth',1.2);
xlabel('t (s)'); ylabel('v (m/s)'); title('Sway'); grid on;

subplot(2,3,3); viz.h_w  = animatedline('Color',[0.17 0.63 0.17],'LineWidth',1.2);
xlabel('t (s)'); ylabel('w (m/s)'); title('Heave'); grid on;

subplot(2,3,4); viz.h_p  = animatedline('Color',[0.84 0.15 0.16],'LineWidth',1.2);
xlabel('t (s)'); ylabel('p (deg/s)'); title('Roll rate'); grid on;

subplot(2,3,5); viz.h_q  = animatedline('Color',[0.58 0.40 0.74],'LineWidth',1.2);
xlabel('t (s)'); ylabel('q (deg/s)'); title('Pitch rate'); grid on;

subplot(2,3,6); viz.h_r  = animatedline('Color',[0.55 0.34 0.29],'LineWidth',1.2);
xlabel('t (s)'); ylabel('r (deg/s)'); title('Yaw rate'); grid on;

viz.fig1 = fig1;

% --- Figure 2: Position & attitude ---
fig2 = figure(11);
set(fig2,'Name','Live — Position','NumberTitle','off', ...
    'Position',[660 560 640 420]);

subplot(2,3,1); viz.h_xn   = animatedline('Color',[0.13 0.47 0.71],'LineWidth',1.2);
xlabel('t (s)'); ylabel('x_N (m)'); title('North'); grid on;

subplot(2,3,2); viz.h_ye   = animatedline('Color',[0.90 0.45 0.10],'LineWidth',1.2);
xlabel('t (s)'); ylabel('y_E (m)'); title('East'); grid on;

subplot(2,3,3); viz.h_zd   = animatedline('Color',[0.17 0.63 0.17],'LineWidth',1.2);
xlabel('t (s)'); ylabel('z_D (m)'); title('Depth'); grid on;

subplot(2,3,4); viz.h_phi  = animatedline('Color',[0.84 0.15 0.16],'LineWidth',1.2);
xlabel('t (s)'); ylabel('phi (deg)'); title('Roll'); grid on;

subplot(2,3,5); viz.h_theta= animatedline('Color',[0.58 0.40 0.74],'LineWidth',1.2);
xlabel('t (s)'); ylabel('theta (deg)'); title('Pitch'); grid on;

subplot(2,3,6); viz.h_psi  = animatedline('Color',[0.55 0.34 0.29],'LineWidth',1.2);
xlabel('t (s)'); ylabel('psi (deg)'); title('Yaw'); grid on;

viz.fig2 = fig2;

% --- Figure 3: Control signals ---
fig3 = figure(12);
set(fig3,'Name','Live — Control','NumberTitle','off', ...
    'Position',[10 80 640 420]);

subplot(3,1,1); viz.h_n_rpm = animatedline('Color',[0.13 0.47 0.71],'LineWidth',1.2);
hold on;
yline(0,'--k','LineWidth',0.5); yline(1525,'--r','LineWidth',0.5,'Label','Max');
xlabel('t (s)'); ylabel('RPM'); title('Propeller speed'); grid on; ylim([-50 1600]);

subplot(3,1,2); viz.h_ds = animatedline('Color',[0.90 0.45 0.10],'LineWidth',1.2);
hold on; yline(20,'--r','LineWidth',0.5); yline(-20,'--r','LineWidth',0.5);
xlabel('t (s)'); ylabel('deg'); title('Stern plane δ_s'); grid on; ylim([-25 25]);

subplot(3,1,3); viz.h_dr = animatedline('Color',[0.17 0.63 0.17],'LineWidth',1.2);
hold on; yline(20,'--r','LineWidth',0.5); yline(-20,'--r','LineWidth',0.5);
xlabel('t (s)'); ylabel('deg'); title('Rudder δ_r'); grid on; ylim([-25 25]);

viz.fig3 = fig3;

% --- Figure 4: Cross-track errors ---
fig4 = figure(13);
set(fig4,'Name','Live — Cross-track errors','NumberTitle','off', ...
    'Position',[660 80 640 420]);

subplot(3,1,1); viz.h_xe = animatedline('Color',[0.13 0.47 0.71],'LineWidth',1.2);
xlabel('t (s)'); ylabel('x_e (m)'); title('Along-track error'); grid on;
yline(0,'-k','LineWidth',0.5);

subplot(3,1,2); viz.h_lye = animatedline('Color',[0.84 0.15 0.16],'LineWidth',1.2);
xlabel('t (s)'); ylabel('y_e (m)'); title('Lateral cross-track error'); grid on;
yline(0,'-k','LineWidth',0.5);

subplot(3,1,3); viz.h_lze = animatedline('Color',[0.17 0.63 0.17],'LineWidth',1.2);
xlabel('t (s)'); ylabel('z_e (m)'); title('Vertical cross-track error'); grid on;
yline(0,'-k','LineWidth',0.5);

viz.fig4 = fig4;

drawnow;
fprintf('viz_init: 4 live-update figures created (figs 10-13).\n');

end

% =========================================================================
function viz = viz_update(viz, log)
%% viz_update  —  Add latest logged point to all live figures
%
% Uses animatedline for efficient incremental updates —
% only the latest point is added, not the entire trace redrawn.

k = log.k;
if k < 1, return, end

t = log.t(k);

% Figure 1 — velocities
addpoints(viz.h_u,  t, log.x(1,k));
addpoints(viz.h_v,  t, log.x(2,k));
addpoints(viz.h_w,  t, log.x(3,k));
addpoints(viz.h_p,  t, rad2deg(log.x(4,k)));
addpoints(viz.h_q,  t, rad2deg(log.x(5,k)));
addpoints(viz.h_r,  t, rad2deg(log.x(6,k)));

% Figure 2 — position/attitude
addpoints(viz.h_xn,    t, log.x(7,k));
addpoints(viz.h_ye,    t, log.x(8,k));
addpoints(viz.h_zd,    t, log.x(9,k));
addpoints(viz.h_phi,   t, rad2deg(log.x(10,k)));
addpoints(viz.h_theta, t, rad2deg(log.x(11,k)));
addpoints(viz.h_psi,   t, rad2deg(log.x(12,k)));

% Figure 3 — control signals
addpoints(viz.h_n_rpm, t, log.n_direct(k));
addpoints(viz.h_ds,    t, rad2deg(log.ui(2,k)));
addpoints(viz.h_dr,    t, rad2deg(log.ui(1,k)));

% Figure 4 — cross-track errors
addpoints(viz.h_xe,  t, log.los_xe(k));
addpoints(viz.h_lye, t, log.los_ye(k));
addpoints(viz.h_lze, t, log.los_ze(k));

drawnow limitrate;   % non-blocking — skips if MATLAB is busy

end

% =========================================================================
function viz_final(log)
%% viz_final  —  Generate complete post-run figure set from log struct
%
% Generates 5 publication-quality figures:
%   Fig 20 — 3D trajectory (vehicle vs reference path)
%   Fig 21 — 6-DOF state time histories
%   Fig 22 — Control signals (RPM, fins) with saturation limits
%   Fig 23 — Cross-track errors vs time
%   Fig 24 — Environment signals (current, wave forces)

if ~isfield(log,'k') || log.k < 2
    fprintf('viz_final: log is empty or has < 2 steps.\n');
    return
end

k  = log.k;
t  = log.t(1:k);
lw = 1.5;  fs = 11;
c  = {[0.13 0.47 0.71],[0.90 0.45 0.10],[0.17 0.63 0.17], ...
      [0.84 0.15 0.16],[0.58 0.40 0.74],[0.55 0.34 0.29]};

% --- Figure 20: 3D trajectory ---
fig20 = figure(20);
set(fig20,'Name','Post-run — 3D trajectory','NumberTitle','off', ...
    'Position',[50 450 720 520]);
plot3(log.x(7,1:k), log.x(8,1:k), -log.x(9,1:k), ...
    'Color',c{1},'LineWidth',lw,'DisplayName','Vehicle'); hold on;
scatter3(log.x(7,1), log.x(8,1), -log.x(9,1), 80,'g','filled','DisplayName','Start');
scatter3(log.x(7,k), log.x(8,k), -log.x(9,k), 80,'r','filled','DisplayName','End');
xlabel('x_N (m)','FontSize',fs); ylabel('y_E (m)','FontSize',fs);
zlabel('Altitude (m)','FontSize',fs);
title('3D AUV trajectory','FontSize',fs);
legend('Location','best','FontSize',9); grid on; axis equal; view(30,25);

% --- Figure 21: 6-DOF states ---
fig21 = figure(21);
set(fig21,'Name','Post-run — 6-DOF states','NumberTitle','off', ...
    'Position',[50 50 1100 700]);
labels_vel = {'u (m/s)','v (m/s)','w (m/s)','p (deg/s)','q (deg/s)','r (deg/s)'};
labels_pos = {'x_N (m)','y_E (m)','z_D (m)','phi (deg)','theta (deg)','psi (deg)'};
scale_ang  = [1 1 1 180/pi 180/pi 180/pi];
for i = 1:6
    subplot(4,3,i)
    plot(t, log.x(i,1:k)*scale_ang(i), 'Color',c{i},'LineWidth',lw);
    xlabel('t (s)','FontSize',9); ylabel(labels_vel{i},'FontSize',9);
    grid on;
end
for i = 1:6
    subplot(4,3,6+i)
    plot(t, log.x(6+i,1:k)*scale_ang(i), 'Color',c{i},'LineWidth',lw);
    xlabel('t (s)','FontSize',9); ylabel(labels_pos{i},'FontSize',9);
    grid on;
    if i >= 4, yline(0,'-k','LineWidth',0.5); end
end
sgtitle('Post-run: All 12 states','FontSize',fs);

% --- Figure 22: Control signals ---
fig22 = figure(22);
set(fig22,'Name','Post-run — Control','NumberTitle','off', ...
    'Position',[780 450 640 450]);
subplot(3,1,1)
plot(t, log.n_direct(1:k),'Color',c{1},'LineWidth',lw);
yline(1525,'--r','LineWidth',1,'Label','Max RPM');
yline(0,'--k','LineWidth',0.5);
xlabel('t(s)','FontSize',fs); ylabel('RPM','FontSize',fs);
title('Propeller speed','FontSize',fs); grid on;

subplot(3,1,2)
plot(t, rad2deg(log.ui(2,1:k)),'Color',c{2},'LineWidth',lw);
yline(20,'--r','LineWidth',1); yline(-20,'--r','LineWidth',1);
xlabel('t(s)','FontSize',fs); ylabel('δ_s (deg)','FontSize',fs);
title('Stern plane','FontSize',fs); grid on;

subplot(3,1,3)
plot(t, rad2deg(log.ui(1,1:k)),'Color',c{3},'LineWidth',lw);
yline(20,'--r','LineWidth',1); yline(-20,'--r','LineWidth',1);
xlabel('t(s)','FontSize',fs); ylabel('δ_r (deg)','FontSize',fs);
title('Rudder','FontSize',fs); grid on;

sgtitle('Post-run: Actuator signals','FontSize',fs);

% --- Figure 23: Cross-track errors ---
fig23 = figure(23);
set(fig23,'Name','Post-run — Cross-track errors','NumberTitle','off', ...
    'Position',[780 50 640 420]);
subplot(3,1,1)
plot(t, log.los_xe(1:k),'Color',c{1},'LineWidth',lw);
yline(0,'-k','LineWidth',0.5); xlabel('t(s)','FontSize',fs);
ylabel('x_e (m)','FontSize',fs); title('Along-track error','FontSize',fs); grid on;

subplot(3,1,2)
plot(t, log.los_ye(1:k),'Color',c{4},'LineWidth',lw);
yline(0,'-k','LineWidth',0.5); xlabel('t(s)','FontSize',fs);
ylabel('y_e (m)','FontSize',fs); title('Lateral cross-track','FontSize',fs); grid on;

subplot(3,1,3)
plot(t, log.los_ze(1:k),'Color',c{3},'LineWidth',lw);
yline(0,'-k','LineWidth',0.5); xlabel('t(s)','FontSize',fs);
ylabel('z_e (m)','FontSize',fs); title('Vertical cross-track','FontSize',fs); grid on;

sgtitle('Post-run: LOS cross-track errors','FontSize',fs);

% --- Figure 24: Environment ---
fig24 = figure(24);
set(fig24,'Name','Post-run — Environment','NumberTitle','off', ...
    'Position',[50 50 640 380]);
subplot(2,2,1)
plot(t, log.Vc(1:k),'Color',c{1},'LineWidth',lw);
xlabel('t(s)','FontSize',fs); ylabel('Vc (m/s)','FontSize',fs);
title('Current speed','FontSize',fs); grid on;

subplot(2,2,2)
plot(t, rad2deg(log.betaVc(1:k)),'Color',c{2},'LineWidth',lw);
xlabel('t(s)','FontSize',fs); ylabel('betaVc (deg)','FontSize',fs);
title('Current direction','FontSize',fs); grid on;

subplot(2,2,3)
plot(t, log.tau_env(3,1:k),'Color',c{3},'LineWidth',lw);
xlabel('t(s)','FontSize',fs); ylabel('tau_Z (N)','FontSize',fs);
title('Wave heave force','FontSize',fs); grid on;

subplot(2,2,4)
plot(t, log.tau_env(5,1:k),'Color',c{4},'LineWidth',lw);
xlabel('t(s)','FontSize',fs); ylabel('tau_M (N·m)','FontSize',fs);
title('Wave pitch moment','FontSize',fs); grid on;

sgtitle('Post-run: Environmental disturbances','FontSize',fs);

fprintf('viz_final: Figures 20-24 generated.\n');

end

% =========================================================================
function viz_export(output_dir, prefix)
%% viz_export  —  Save all open simulation figures to PNG files

if nargin < 1, output_dir = '.'; end
if nargin < 2, prefix = 'auv';  end

fig_ids = [10 11 12 13 20 21 22 23 24];
names   = {'velocities','position','control','xtrack_live', ...
           'trajectory_3d','states_6dof','actuators', ...
           'crosstrack','environment'};

for i = 1:numel(fig_ids)
    fid = fig_ids(i);
    if ishandle(fid)
        fname = fullfile(output_dir, sprintf('%s_%s.png', prefix, names{i}));
        exportgraphics(figure(fid), fname, 'Resolution', 150);
        fprintf('Saved: %s\n', fname);
    end
end

end