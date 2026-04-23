%% view_run.m  —  Post-Simulation Log Viewer
% Automatically finds the most recent AUV log file, prints the summary,
% and generates all post-run figures.
%
% HOW TO RUN:
%   >> view_run

clear log; % Clear old logs from memory
close all; % Close old live figures

% 1. Find the newest log file in the directory
files = dir('auv_log_*.mat');
if isempty(files)
    error('No log files found in the current directory.');
end
[~, idx] = max([files.datenum]);
newest_file = files(idx).name;

fprintf('\nLoading latest log: %s...\n', newest_file);
load(newest_file, 'log');

% 2. Call the local functions to summarize and plot
log_summary_local(log);
viz_final_local(log);

% =========================================================================
% LOCAL FUNCTIONS (Bypassing MATLAB scope limits)
% =========================================================================

function s = get_style_local()
% Shared style constants — keep in sync with viz_lib.m and run_simulation.m
s.lw_act=1.5; s.lw_ref=1.2; s.lw_lim=0.8; s.lw_zero=0.5;
s.fs_sgt=12;  s.fs_ttl=10;  s.fs_lbl=9;   s.fs_lgn=8;
s.c={[0.13 0.47 0.71],[0.90 0.45 0.10],[0.17 0.63 0.17], ...
     [0.84 0.15 0.16],[0.58 0.40 0.74],[0.55 0.34 0.29]};
end

function log_summary_local(log)
k = log.k;
if k == 0, fprintf('Log empty.\n'); return; end
t_end = log.t(k);
fprintf('\n=== Simulation Log Summary ===\n');
fprintf('  Duration:      %.1f s  (%d steps)\n', t_end, k);
fprintf('  Max |u|:       %.3f m/s\n', max(abs(log.x(1,1:k))));
fprintf('  Max |v|:       %.3f m/s\n', max(abs(log.x(2,1:k))));
fprintf('  Max |w|:       %.3f m/s\n', max(abs(log.x(3,1:k))));
fprintf('  North range:   [%.1f, %.1f] m\n', min(log.x(7,1:k)), max(log.x(7,1:k)));
fprintf('  East  range:   [%.1f, %.1f] m\n', min(log.x(8,1:k)), max(log.x(8,1:k)));
fprintf('  Depth range:   [%.2f, %.2f] m\n', min(log.x(9,1:k)), max(log.x(9,1:k)));
if any(log.los_ye(1:k) ~= 0)
    rms_ye = sqrt(mean(log.los_ye(1:k).^2));
    rms_ze = sqrt(mean(log.los_ze(1:k).^2));
    fprintf('  RMS lateral x-track:   %.2f m\n', rms_ye);
    fprintf('  RMS vertical x-track:  %.2f m\n', rms_ze);
end
sat_total = sum(log.sat_flags(:,1:k) > 0, 'all');
fprintf('  Saturation events: %d\n', sat_total);
if any(isnan(log.x(:,1:k)),'all') || any(isinf(log.x(:,1:k)),'all')
    fprintf('  WARNING: NaN or Inf detected in state log!\n');
else
    fprintf('  State integrity: OK (no NaN/Inf)\n');
end
fprintf('==============================\n\n');
end

function viz_final_local(log)
if ~isfield(log,'k') || log.k < 2, return; end
s  = get_style_local();
k  = log.k;  t = log.t(1:k);
z_ref = log.x(9,1:k) - log.los_ze(1:k);

% --- Figure 20: 3D Trajectory ---
fig20 = figure(20);
set(fig20,'Name','Post-run — 3D Trajectory','NumberTitle','off','Position',[50 430 720 520]);
clf(fig20);
plot3(log.x(7,1:k), log.x(8,1:k), -log.x(9,1:k), ...
    'Color',s.c{1},'LineWidth',s.lw_act,'DisplayName','Vehicle path');
hold on;
scatter3(log.x(7,1), log.x(8,1), -log.x(9,1), 100,'g','filled','DisplayName','Start');
scatter3(log.x(7,k), log.x(8,k), -log.x(9,k), 100,'r','filled','DisplayName','End');
hold off;
xlabel('North (m)','FontSize',s.fs_lbl); ylabel('East (m)','FontSize',s.fs_lbl);
zlabel('Altitude — up+ve (m)','FontSize',s.fs_lbl);
title(sprintf('3D AUV Trajectory  [T = %.0f s,  d = %.0f m]', ...
    t(k), norm(log.x(7:9,k)-log.x(7:9,1))),'FontSize',s.fs_ttl);
legend('FontSize',s.fs_lgn,'Location','best');
grid on; daspect([1 1 0.05]); view(30,25);

% --- Figure 21: All 12 States ---
fig21 = figure(21);
set(fig21,'Name','Post-run — All 12 States','NumberTitle','off','Position',[50 30 1140 720]);
clf(fig21);
vel_titles  = {'Surge u','Sway v','Heave w','Roll Rate p','Pitch Rate q','Yaw Rate r'};
vel_ylabels = {'u (m/s)','v (m/s)','w (m/s)','p (deg/s)','q (deg/s)','r (deg/s)'};
for i = 1:6
    ax = subplot(4,3,i); hold(ax,'on');
    switch i
        case 1
            h1 = plot(t, log.ud(1:k),'--k','LineWidth',s.lw_ref,'DisplayName','u_d');
            h2 = plot(t, log.x(1,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','u');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
        case {2,3}
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
            plot(t, log.x(i,1:k),'Color',s.c{i},'LineWidth',s.lw_act);
        case 4
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
            plot(t, rad2deg(log.x(i,1:k)),'Color',s.c{i},'LineWidth',s.lw_act);
        case {5,6}
            plot(t, rad2deg(log.x(i,1:k)),'Color',s.c{i},'LineWidth',s.lw_act);
    end
    title(vel_titles{i},'FontSize',s.fs_ttl);
    xlabel('t (s)','FontSize',s.fs_lbl); ylabel(vel_ylabels{i},'FontSize',s.fs_lbl);
    grid on; hold(ax,'off');
end
pos_titles  = {'North x_N','East y_E','Depth z_D','Roll \phi','Pitch \theta','Yaw \psi'};
pos_ylabels = {'x_N (m)','y_E (m)','z_D (m)','\phi (deg)','\theta (deg)','\psi (deg)'};
for i = 1:6
    ax = subplot(4,3,6+i); hold(ax,'on');
    switch i
        case {1,2}
            plot(t, log.x(6+i,1:k),'Color',s.c{i},'LineWidth',s.lw_act);
        case 3
            h1 = plot(t, z_ref,'--k','LineWidth',s.lw_ref,'DisplayName','z_{ref}');
            h2 = plot(t, log.x(9,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','z_D');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
        case 4
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
            plot(t, rad2deg(log.x(10,1:k)),'Color',s.c{i},'LineWidth',s.lw_act);
        case 5
            h1 = plot(t, rad2deg(log.upsilon_d(1:k)),'--k','LineWidth',s.lw_ref,'DisplayName','\upsilon_d');
            h2 = plot(t, rad2deg(log.x(11,1:k)),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','\theta');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
        case 6
            h1 = plot(t, rad2deg(log.chi_d(1:k)),'--k','LineWidth',s.lw_ref,'DisplayName','\chi_d');
            h2 = plot(t, rad2deg(log.x(12,1:k)),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','\psi');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
    end
    title(pos_titles{i},'FontSize',s.fs_ttl);
    xlabel('t (s)','FontSize',s.fs_lbl); ylabel(pos_ylabels{i},'FontSize',s.fs_lbl);
    grid on; hold(ax,'off');
end
sgtitle('Post-run: All 12 States — Desired vs Actual','FontSize',s.fs_sgt);

% --- Figure 22: Actuator Signals ---
fig22 = figure(22);
set(fig22,'Name','Post-run — Actuator Signals','NumberTitle','off','Position',[800 430 640 460]);
clf(fig22);
subplot(3,1,1);
plot(t, log.n_direct(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
hold on;
yline(1525,'--r','LineWidth',s.lw_lim,'Label','Max 1525 RPM','HandleVisibility','off');
yline(0,'--k','LineWidth',s.lw_zero,'HandleVisibility','off');
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

% --- Figure 23: LOS Cross-track Errors ---
fig23 = figure(23);
set(fig23,'Name','Post-run — LOS Cross-track Errors','NumberTitle','off','Position',[800 30 640 440]);
clf(fig23);
rms_xe = sqrt(mean(log.los_xe(1:k).^2));
rms_ye = sqrt(mean(log.los_ye(1:k).^2));
rms_ze = sqrt(mean(log.los_ze(1:k).^2));
subplot(3,1,1);
plot(t, log.los_xe(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Along-track Error x_e    [RMS = %.2f m]', rms_xe),'FontSize',s.fs_ttl);
ylabel('x_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl); grid on;
subplot(3,1,2);
plot(t, log.los_ye(1:k),'Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Lateral Cross-track y_e    [RMS = %.2f m]', rms_ye),'FontSize',s.fs_ttl);
ylabel('y_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl); grid on;
subplot(3,1,3);
plot(t, log.los_ze(1:k),'Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Vertical Cross-track z_e    [RMS = %.2f m]', rms_ze),'FontSize',s.fs_ttl);
ylabel('z_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl); grid on;
sgtitle('Post-run: LOS Cross-track Errors','FontSize',s.fs_sgt);

% --- Figure 24: Environmental Disturbances ---
fig24 = figure(24);
set(fig24,'Name','Post-run — Environmental Disturbances','NumberTitle','off','Position',[50 30 700 420]);
clf(fig24);
subplot(2,2,1);
plot(t, log.Vc(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
title('Ocean Current Speed','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('V_c (m/s)','FontSize',s.fs_lbl); grid on;
subplot(2,2,2);
plot(t, rad2deg(log.betaVc(1:k)),'Color',s.c{2},'LineWidth',s.lw_act);
title('Current Direction','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\beta_{Vc} (deg)','FontSize',s.fs_lbl); grid on;
subplot(2,2,3);
plot(t, log.tau_env(3,1:k),'Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Wave Heave Force \tau_Z','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\tau_Z (N)','FontSize',s.fs_lbl); grid on;
subplot(2,2,4);
plot(t, log.tau_env(5,1:k),'Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Wave Pitch Moment \tau_M','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\tau_M (N·m)','FontSize',s.fs_lbl); grid on;
sgtitle('Post-run: Environmental Disturbances','FontSize',s.fs_sgt);

fprintf('\n  Post-run figures 20-24 generated.\n');
fprintf('  RMS cross-track:  x_e=%.3fm  y_e=%.3fm  z_e=%.3fm\n', rms_xe, rms_ye, rms_ze);
end