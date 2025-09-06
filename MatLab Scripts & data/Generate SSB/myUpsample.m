%% Upsampling code for loaded Signal
clear;clc;close all;
%%
upsample_factor = 9; % signal will be k times longer!

%% Load QPSK signal
load("QPSK_signal.mat");
loaded_signal = QPSK_signal;

%% Load SSB signal
load("pilotSSB.mat");
%loaded_signal = cleanSignal;

%% Test Signal

%loaded_signal = floor(rand(1, 100)*10);

%% Upsample & Prepare samples index

up_signal = upsample(loaded_signal, upsample_factor);

index_points = reshape(1:length(up_signal), [upsample_factor, length(loaded_signal)]);

provided_points = index_points(1,:);

query_points = index_points(2:end,:);
query_points = reshape(query_points, [1 numel(query_points)]);

%% Interp1D & Fill UpSignal

Interpolated_vals = interp1(provided_points, loaded_signal, query_points);

up_signal(query_points) = Interpolated_vals;

%% Verify Interpolation

figure;stem(provided_points, real(loaded_signal), "bO");hold on;

plot(real(up_signal),"r", "LineWidth",  1.5);legend("Original", "Interp");hold off;