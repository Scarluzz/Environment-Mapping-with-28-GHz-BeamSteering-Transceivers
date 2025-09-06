clear;clc;close all

%% SImple script to generate reference signal plots


load("pilotSSB64.mat");

realPart = real(cleanSignal);

imagPart = imag(cleanSignal);

Fs = 61440000;

time = 1:length(cleanSignal); %%time = time / Fs;

figure;
subplot(1, 2, 1);
plot(time, realPart, "b"); hold on;
plot(time, imagPart, "r"); hold off;
title('Generated SSB signal sequence');subtitle('Full 64 Allocated SSBs')
xlabel('Samples');
ylabel('Amplitude');
legend("Real", "Imaginary");


% Zoom on a subpart

subplot(1,2,2);
plot(time, realPart, "b"); hold on;
plot(time, imagPart, "r"); hold off;
xlim([35000,85000]);
title('Generated SSB signal sequence');subtitle('Zoom')
xlabel('Samples');
ylabel('Amplitude');
legend("Real", "Imaginary");


%% Same for CSI resources

load("pilotCSI.mat");

realPart = real(cleanSignal);

imagPart = imag(cleanSignal);

Fs = 61440000;

time = 1:length(cleanSignal); %%time = time / Fs;

figure;
subplot(1, 2, 1);
plot(time, realPart, "b"); hold on;
plot(time, imagPart, "r"); hold off;
title('Generated CSI signal sequence');subtitle('Allocated 10 Resources')
xlabel('Samples');
ylabel('Amplitude');
legend("Real", "Imaginary");


% Zoom on a subpart

subplot(1,2,2);
plot(time, realPart, "b"); hold on;
plot(time, imagPart, "r"); hold off;
xlim([1,35000]);
title('Generated CSI signal sequence');subtitle('Zoom')
xlabel('Samples');
ylabel('Amplitude');
legend("Real", "Imaginary");