%% plot_phase2.m  —  Phase 2 Diagnostic Plots
%
% PURPOSE:
%   Generates the four standard Phase 2 diagnostic figures from the
%   data saved by test_phase2_openloop.m.
%   Run AFTER test_phase2_openloop passes all [PASS] checks.
%
% FIGURES PRODUCED:
%   Figure 1 — Surge acceleration: u(t) for Test 3 (1525 RPM open-loop)
%   Figure 2 — 6-DOF state overview: all 12 states for Test 3
%   Figure 3 — Rudder step response: yaw and sway (Test 5)
%   Figure 4 — Stern plane response: pitch and depth (Test 6)
%
% HOW TO RUN:
%   >> plot_phase2          (uses phase2_results.mat)
%   >> plot_phase2(t,X,...) (pass data directly — see usage below)
%
% AUTHOR: AUV Simulation Project — Phase 2

if ~exist('phase2_results.mat','file')
    error('Run test_phase2_openloop first — phase2_results.mat not found.');
end
load('phase2_results.mat');

% =========================================================================
% Shared style
% =========================================================================
lw  = 1.5;                   % line width
fs  = 11;                    % font size
col_surge  = [0.13 0.47 0.71];   % blue
col_sway   = [0.90 0.45 0.10];   % orange
col_heave  = [0.17 0.63 0.17];   % green
col_yaw    = [0.84 0.15 0.16];   % red
col_pitch  = [0.58 0.40 0.74];   % purple
col_ref    = [0.5 0.5 0.5];      % grey for limits

close all;

% =========================================================================
% Figure 1 — Surge acceleration profile
% =========================================================================
fig1 = figure(1);
set(fig1,'Name','Phase 2 — Surge acceleration (1525 RPM)','NumberTitle','off', ...
    'Position',[50 500 700 380]);

subplot(2,1,1)
plot(t3, X3(:,1), 'Color', col_surge, 'LineWidth', lw); hold on;
yline(2.5, '--', 'Color', col_ref, 'LineWidth', 1, 'Label', 'Max spec 2.5 m/s');
yline(2.0, ':', 'Color', col_ref, 'LineWidth', 1, 'Label', 'Min gate 2.0 m/s');
xlabel('Time (s)', 'FontSize', fs);
ylabel('u  (m/s)', 'FontSize', fs);
title('Surge velocity — 1525 RPM open loop', 'FontSize', fs);
grid on; xlim([0 60]); ylim([0 3]);

subplot(2,1,2)
udot = gradient(X3(:,1), t3);
plot(t3, udot, 'Color', col_surge, 'LineWidth', lw);
xlabel('Time (s)', 'FontSize', fs);
ylabel('du/dt  (m/s²)', 'FontSize', fs);
title('Surge acceleration (numerical derivative)', 'FontSize', fs);
grid on; xlim([0 60]);
yline(0, '-k', 'LineWidth', 0.5);

% =========================================================================
% Figure 2 — Full 6-DOF state overview (Test 3: surge run)
% =========================================================================
fig2 = figure(2);
set(fig2,'Name','Phase 2 — 6-DOF state overview','NumberTitle','off', ...
    'Position',[50 50 1000 700]);

state_labels = {'u (m/s)','v (m/s)','w (m/s)','p (rad/s)','q (rad/s)','r (rad/s)', ...
                'x_N (m)','y_E (m)','z_D (m)','phi (deg)','theta (deg)','psi (deg)'};
scale = [1 1 1 1 1 1 1 1 1 180/pi 180/pi 180/pi];  % convert angles to deg

colors = {col_surge, col_sway, col_heave, col_yaw, col_pitch, col_ref, ...
          col_surge, col_sway, col_heave, col_yaw, col_pitch, col_ref};

for k = 1:12
    subplot(4,3,k)
    plot(t3, X3(:,k)*scale(k), 'Color', colors{k}, 'LineWidth', lw);
    xlabel('t (s)', 'FontSize', 9);
    ylabel(state_labels{k}, 'FontSize', 9);
    grid on; xlim([0 60]);
    if k >= 10  % angle states
        yline(0, '-k', 'LineWidth', 0.5);
    end
end
sgtitle('Phase 2: All 12 states — 1525 RPM, zero fins, 60s', 'FontSize', fs);

% =========================================================================
% Figure 3 — Rudder step response (Test 5)
% =========================================================================
fig3 = figure(3);
set(fig3,'Name','Phase 2 — Rudder step response','NumberTitle','off', ...
    'Position',[760 500 700 380]);

subplot(2,2,1)
plot(t5, rad2deg(X5(:,12)), 'Color', col_yaw, 'LineWidth', lw);
xlabel('t (s)','FontSize',fs); ylabel('psi (deg)','FontSize',fs);
title('Yaw angle','FontSize',fs); grid on;

subplot(2,2,2)
plot(t5, rad2deg(X5(:,6)), 'Color', col_yaw, 'LineWidth', lw);
xlabel('t (s)','FontSize',fs); ylabel('r (deg/s)','FontSize',fs);
title('Yaw rate','FontSize',fs); grid on;

subplot(2,2,3)
plot(t5, X5(:,2), 'Color', col_sway, 'LineWidth', lw);
xlabel('t (s)','FontSize',fs); ylabel('v (m/s)','FontSize',fs);
title('Sway velocity','FontSize',fs); grid on;

subplot(2,2,4)
plot(t5, X5(:,8), 'Color', col_sway, 'LineWidth', lw); hold on;
plot(t5, X5(:,7), 'Color', col_surge, 'LineWidth', lw, 'LineStyle','--');
xlabel('t (s)','FontSize',fs); ylabel('position (m)','FontSize',fs);
title('Horizontal trajectory (x=N, y=E)','FontSize',fs);
legend('y_E','x_N','Location','northwest','FontSize',9); grid on;

sgtitle(sprintf('Phase 2: Rudder step \\delta_r = %.0f deg, u_0 = 1.5 m/s', ...
    rad2deg(deg2rad(15))), 'FontSize', fs);

% =========================================================================
% Figure 4 — Stern plane step response (Test 6)
% =========================================================================
fig4 = figure(4);
set(fig4,'Name','Phase 2 — Stern plane step response','NumberTitle','off', ...
    'Position',[760 50 700 380]);

subplot(2,2,1)
plot(t6, rad2deg(X6(:,11)), 'Color', col_pitch, 'LineWidth', lw);
xlabel('t (s)','FontSize',fs); ylabel('theta (deg)','FontSize',fs);
title('Pitch angle','FontSize',fs); grid on;
yline(0,'-k','LineWidth',0.5);

subplot(2,2,2)
plot(t6, rad2deg(X6(:,5)), 'Color', col_pitch, 'LineWidth', lw);
xlabel('t (s)','FontSize',fs); ylabel('q (deg/s)','FontSize',fs);
title('Pitch rate','FontSize',fs); grid on;

subplot(2,2,3)
plot(t6, X6(:,9), 'Color', col_heave, 'LineWidth', lw);
xlabel('t (s)','FontSize',fs); ylabel('z_D (m)  +ve=down','FontSize',fs);
title('Depth (NED down is +ve)','FontSize',fs); grid on;
yline(0,'-k','LineWidth',0.5);

subplot(2,2,4)
plot(t6, X6(:,3), 'Color', col_heave, 'LineWidth', lw);
xlabel('t (s)','FontSize',fs); ylabel('w (m/s)','FontSize',fs);
title('Heave velocity','FontSize',fs); grid on;
yline(0,'-k','LineWidth',0.5);

sgtitle(sprintf('Phase 2: Stern plane step \\delta_s = 10 deg, u_0 = 1.5 m/s'), ...
    'FontSize', fs);

fprintf('\nPhase 2 plots generated (Figures 1–4).\n');
fprintf('Expected observations:\n');
fprintf('  Fig 1: Surge rises smoothly to ~2.4 m/s, no oscillation\n');
fprintf('  Fig 2: All non-surge states remain near zero (open loop)\n');
fprintf('  Fig 3: Rudder produces yaw + sway coupling — turning circle\n');
fprintf('  Fig 4: Stern plane produces pitch + depth change — dive/climb\n\n');
fprintf('If all observations match → Phase 2 COMPLETE → proceed to Phase 3\n\n');
