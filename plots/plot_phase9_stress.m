%% plot_phase9_stress.m — Visualizing the SMC vs PID Stress Test

if ~exist('phase9_stress_results.mat','file')
    error('Run test_phase9_stress first.');
end
load('phase9_stress_results.mat');

c_pid = [0.13 0.47 0.71]; % Blue
c_smc = [0.84 0.15 0.16]; % Red
c_ref = [0.5 0.5 0.5];    % Grey
lw = 1.5; fs = 11;
close all;

% --- Figure 1: 2D Spatial Path (Crab Angle Visualization) ---
figure(1); set(gcf,'Position',[100 100 600 500],'Name','Spatial Path');
plot(X_pid(:,8), X_pid(:,7), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID Path'); hold on;
plot(X_smc(:,8), X_smc(:,7), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC Path');
xline(0, '--', 'Color', c_ref, 'LineWidth', 2, 'DisplayName', 'Desired Path (North)');
xlabel('East (y_E) [m]', 'FontSize', fs); ylabel('North (x_N) [m]', 'FontSize', fs);
title('Top-Down View under 0.8 m/s Eastern Current', 'FontSize', fs+2);
legend('Location','northwest'); grid on;
xlim([-40 20]); % Zoom in to show the lateral drift

% --- Figure 2: Lateral Cross-Track Error ---
figure(2); set(gcf,'Position',[750 100 600 500],'Name','Cross-Track Error');
plot(t_pid, ye_pid, 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID y_e'); hold on;
plot(t_smc, ye_smc, 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC y_e');
yline(0, '--k', 'LineWidth', 1);
xlabel('Time [s]', 'FontSize', fs); ylabel('Cross-track error y_e [m]', 'FontSize', fs);
title('Lateral Drift Resistance', 'FontSize', fs+2);
legend('Location','southeast'); grid on;

% --- Figure 3: Heading (Psi) and Crab Angle ---
figure(3); set(gcf,'Position',[100 650 600 400],'Name','Heading & Crab Angle');
plot(t_pid, rad2deg(X_pid(:,12)), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID Heading (\psi)'); hold on;
plot(t_smc, rad2deg(X_smc(:,12)), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC Heading (\psi)');
yline(0, '--k', 'LineWidth', 1, 'DisplayName', 'Path Direction (North = 0°)');
xlabel('Time [s]', 'FontSize', fs); ylabel('Heading Angle [deg]', 'FontSize', fs);
title('Heading Angle Adjustment (Crab Angle)', 'FontSize', fs+2);
legend('Location','best'); grid on;

% --- Figure 4: Rudder Actuation (Wave Disturbance Rejection) ---
figure(4); set(gcf,'Position',[750 650 600 400],'Name','Rudder Actuation');
plot(t_pid, rad2deg(ui_pid(1,:)), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID Rudder (\delta_r)'); hold on;
plot(t_smc, rad2deg(ui_smc(1,:)), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC Rudder (\delta_r)');
yline(20, '--r', 'LineWidth', 1, 'HandleVisibility','off');
yline(-20, '--r', 'LineWidth', 1, 'HandleVisibility','off');
xlabel('Time [s]', 'FontSize', fs); ylabel('Rudder Angle [deg]', 'FontSize', fs);
title('Rudder Effort under 1.5m Waves', 'FontSize', fs+2);
legend('Location','best'); grid on;

% Summary Printout
fprintf('\n--- STRESS TEST RESULTS ---\n');
fprintf('RMS Cross-Track Error (y_e):\n');
fprintf('  PID: %.2f meters\n', sqrt(mean(ye_pid.^2)));
fprintf('  SMC: %.2f meters\n', sqrt(mean(ye_smc.^2)));