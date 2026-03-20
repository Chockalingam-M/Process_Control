% ============================================================
%  plot_io_signals.m
%  Plot input x(n), desired d(n) and filter output y(n)
%  from LMS adaptive filter simulation CSV log.
%
%  Usage:
%    1. Run vvp sim.out   →  generates sim_log.csv
%    2. >> plot_io_signals
% ============================================================

clear; clc; close all;

%% Load CSV
T      = readtable('sim_log.csv');
n      = T.n;
x_in   = double(T.x_in);
d_in   = double(T.d_in);
y_out  = double(T.y_out);

%% Plot
figure('Name','LMS Filter — Input vs Output', ...
       'NumberTitle','off','Position',[100 100 1000 500]);

% --- Input signal x(n) ---
subplot(2,1,1);
plot(n, x_in, 'Color',[0.10 0.45 0.85], 'LineWidth', 1.0);
hold on;
plot(n, d_in, 'Color',[0.18 0.55 0.34], 'LineWidth', 1.2, 'LineStyle','--');
ylabel('Amplitude');
title('Input Signal');
legend('x(n)  — noisy input', 'd(n)  — desired (clean)', 'Location','best');
grid on;
xlim([0 max(n)]);

% --- Output signal y(n) vs desired d(n) ---
subplot(2,1,2);
plot(n, d_in,  'Color',[0.18 0.55 0.34], 'LineWidth', 1.2, 'LineStyle','--');
hold on;
plot(n, y_out, 'Color',[0.85 0.33 0.10], 'LineWidth', 1.0);
ylabel('Amplitude');
xlabel('Sample index  n');
title('Filter Output  y(n)  vs  Desired  d(n)');
legend('d(n)  — desired', 'y(n)  — filter output', 'Location','best');
grid on;
xlim([0 max(n)]);

sgtitle('16-Tap LMS Adaptive Filter — Input / Output', 'FontWeight','bold');