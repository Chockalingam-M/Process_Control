clc;
clear;
close all;

data = readtable('mcu_output.csv');

sample = data.sample;
time = data.time;
x = data.x_in;
d = data.d_in;
y = data.y_out;
e = data.e_out;

% BASIC PARAMETERS
Fs = 10000;
N = length(time);


% MEAN SQUARE ERROR

mse = mean(e.^2);

fprintf('\n');
fprintf('=====================================\n');
fprintf(' MICROCONTROLLER LMS ANALYSIS\n');
fprintf('=====================================\n');

fprintf('Total Samples      : %d\n',N);
fprintf('Mean Square Error  : %f\n',mse);
fprintf('=====================================\n');

% RMS ERROR

rms_error = rms(e);
fprintf('RMS Error          : %f\n',rms_error);


% SIGNAL CORRELATION
corr_val = corr(d,y);
fprintf('Correlation(d,y)   : %f\n',corr_val);



% MAIN SIGNAL PLOTS

figure('Name','Microcontroller LMS Analysis');

% INPUT SIGNAL

subplot(4,1,1);
plot(time,x,'b');
title('Noisy Input Signal x(n)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% DESIRED SIGNAL

subplot(4,1,2);
plot(time,d,'r');
title('Desired Signal d(n)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;


% ADAPTIVE OUTPUT

subplot(4,1,3);
plot(time,y,'m');
title('Adaptive Output y(n)');
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

figure('Name','Phase Comparison');
plot(time,d,'r','LineWidth',2);
hold on;
plot(time,y,'b--','LineWidth',1.5);
xlim([0 0.1]);
title('Desired vs Adaptive Output');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Desired d(n)','Adaptive Output y(n)');
grid on;

% PHASE LAG ESTIMATION

[c,lags] = xcorr(y,d);
[~,idx] = max(c);
lag_samples = lags(idx);
lag_time = lag_samples/Fs;
fprintf('Estimated Phase Delay = %f sec\n',lag_time);

window = 100;
mse_running = movmean(e.^2,window);


% FINAL REPORT

fprintf('\n');
fprintf('=====================================\n');
fprintf(' FINAL RESULTS\n');
fprintf('=====================================\n');
fprintf('MSE                : %f\n',mse);
fprintf('RMS Error          : %f\n',rms_error);
fprintf('Correlation        : %f\n',corr_val);
fprintf('=====================================\n');


Result : Remove this while Running  
=====================================
 MICROCONTROLLER LMS ANALYSIS
=====================================
Total Samples      : 9326
Mean Square Error  : 0.015693
=====================================
RMS Error          : 0.125271
Correlation(d,y)   : 0.984751
Estimated Phase Delay = -0.000200 sec

=====================================
 FINAL RESULTS
=====================================
MSE                : 0.015693
RMS Error          : 0.125271
Correlation        : 0.984751
=====================================
