clc;
clear;
close all;

data = readtable('fpga_parallel_pipeline_output.csv');


% EXTRACT SIGNALS

sample = data.sample;
time = data.time_s;
x = data.x_in_V;
d = data.d_in_V;
y = data.y_out_V;
e = data.e_out_V;


Fs = 10000;
N = length(time);

fprintf('\n');

fprintf('=====================================\n');
fprintf(' FPGA LMS ANALYSIS\n');
fprintf('=====================================\n');

fprintf('Total Samples      : %d\n',N);

% MEAN SQUARE ERROR
mse = mean(e.^2);

% RMS ERROR
rms_error = rms(e);

% SIGNAL CORRELATION
corr_val = corr(d,y);

% SIGNAL TO NOISE RATIO

signal_power = mean(d.^2);
noise_power = mean((d-y).^2);
snr_out = 10*log10(signal_power/noise_power);


% MAIN SIGNAL PLOTS

figure('Name','FPGA LMS Analysis');

% INPUT SIGNAL

subplot(4,1,1);
plot(time,x,'b');
title('FPGA Noisy Input Signal x(n)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% DESIRED SIGNAL

subplot(4,1,2);
plot(time,d,'r');
title('Desired Clean Signal d(n)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% FILTER OUTPUT

subplot(4,1,3);
plot(time,y,'m');
title('FPGA Adaptive Output y(n)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% ERROR SIGNAL

subplot(4,1,4);
plot(time,e,'m');
title('Error Signal e(n)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% PHASE COMPARISON

figure('Name','Desired vs FPGA Output');
plot(time,d,'r','LineWidth',2);
hold on;
plot(time,y,'b--','LineWidth',1.5);
xlim([0 0.1]);
title('Desired Signal vs FPGA Adaptive Output');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Desired d(n)','FPGA Output y(n)');
grid on;

% FINAL REPORT

fprintf('\n');
fprintf('=====================================\n');
fprintf(' FINAL FPGA RESULTS\n');
fprintf('=====================================\n');
fprintf('MSE                : %f\n',mse);
fprintf('RMS Error          : %f\n',rms_error);
fprintf('Correlation        : %f\n',corr_val);
fprintf('Output SNR         : %f dB\n',snr_out);

fprintf('=====================================\n');


% FPGA PERFORMANCE INTERPRETATION

fprintf('\n');
if(corr_val > 0.95)
    fprintf('Excellent convergence achieved.\n');
elseif(corr_val > 0.85)
    fprintf('Good convergence achieved.\n');
else
    fprintf('Poor convergence.\n');
end

fprintf('\n');
fprintf('FPGA LMS analysis completed.\n');
fprintf('=====================================\n');





Result :

=====================================
 FPGA LMS ANALYSIS
=====================================
Total Samples      : 10000

=====================================
 FINAL FPGA RESULTS
=====================================
MSE                : 0.008731
RMS Error          : 0.093442
Correlation        : 0.991289
Output SNR         : 17.442210 dB
=====================================

Excellent convergence achieved.

FPGA LMS analysis completed.
=====================================
