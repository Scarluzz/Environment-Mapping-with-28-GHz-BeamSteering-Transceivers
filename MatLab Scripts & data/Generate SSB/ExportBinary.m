%%
%close all; clear; clc
%% 
%% Get the proper file containing the SSB Burst
NrOfSSB_Beams = 10;
signalConfig = "SSBs_signal_" + NrOfSSB_Beams;
Starters = "starts_" + NrOfSSB_Beams;
LengthFile = "length_" + NrOfSSB_Beams;
% 
% NrOfCSI_Beams = 10;
% signalConfig = "CSI_signal_" + NrOfCSI_Beams;
% Starters = "CSIstarts_" + NrOfCSI_Beams;
% LengthFile = "CSIlength_" + NrOfCSI_Beams;

%%
load(signalConfig);load(Starters);load(LengthFile);

% SSBs_signal = CSI_signal;
% length_SSB = length_CSI;
% NrOfSSB_Beams = NrOfCSI_Beams;

% Get the time-domain Waveform & plot
usrp_data = single(SSBs_signal.waveform);
usrp_data = usrp_data.';

figure;plot(real(usrp_data), "b");hold on;
plot(imag(usrp_data), "r"); hold off;
title('USRP Data Signal : BEFORE REARRANGING');
xlabel('Sample Index');
ylabel('Amplitude');
legend("I", "Q");

%% ReArrange the signal into blocks of packets separated properly
% n = [0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18];
% firstSymbolIndex = [4; 8; 16; 20] + 28*n;
% firstSymbolIndex = firstSymbolIndex(:).';
% div = floor(64/NrOfSSB_Beams);
% candidates = 1:div:64;
% selected = candidates(1:NrOfSSB_Beams);
% StartPacketIndex = firstSymbolIndex(selected);
% StartIndex = 2207; 
% StopIndex = 4416;

samplerate = SSBs_signal.Fs;

if(samplerate == 61440000)
    TotalPacketLength = 10000;
elseif (samplerate == 61440000*2)
        TotalPacketLength = 20000;
else
    "SAMPLERATE NOT STATED"
end

cleanSignal = zeros(NrOfSSB_Beams, TotalPacketLength, "single");

for i = 1:NrOfSSB_Beams
    cleanSignal(i, 1:length_SSB+1) = usrp_data(starts(i):starts(i) + length_SSB);
end

repet = cleanSignal;

for j = 1:5
    cleanSignal = [cleanSignal  ; repet];
end

cleanSignal = cleanSignal.';

cleanSignal = reshape(cleanSignal, [1, numel(cleanSignal)]);

%% Verify if the packets have been correctly rearranged
correctPkts = 0;
for i = 1:NrOfSSB_Beams
    if(isequal(cleanSignal((i-1)*TotalPacketLength + 1:(i-1)*TotalPacketLength + 1+length_SSB), usrp_data(starts(i):starts(i) + length_SSB)))
        correctPkts = correctPkts + 1;
    end
end

if(correctPkts ~= NrOfSSB_Beams)
    "ERROR IN REARRANGING"
end

figure;plot(real(cleanSignal), "b");hold on;
plot(imag(cleanSignal), "r"); hold off;
title('USRP Data Signal : AFTER REARRANGING');
xlabel('Sample Index');
ylabel('Amplitude');
legend("I", "Q");

%% Export Interleaved signal as contigous fc32 I->-Q samples in C++

interleaved_signal = zeros(1, length(cleanSignal)*2, "single");
interleaved_signal(1:2:end) = real(cleanSignal);
interleaved_signal(2:2:end) = imag(cleanSignal);

%% Export Interleaved signal as contigous fc32 I->-Q samples in C++

fid = fopen('usrp_samples.dat', 'w');
fwrite(fid, interleaved_signal, 'float32');
fclose(fid);

size_of_file = {length(interleaved_signal)*4/1000, "KiloBytes long"}
