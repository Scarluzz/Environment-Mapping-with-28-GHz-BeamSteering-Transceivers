%% 
close all;
clear;
clc;

%% Start Generating SSBlock Signal & Properties
% 
% ssb = nrWavegenSSBurstConfig;
% 
% ssb.Power = 0; % By default scales the SSBs to 0 dB value...
% 
% ssb.BlockPattern = "Case D";
% 
% L_Max = 64; % 64 Blocks per SSBurst in FR2 
% 
% BitmapMatrix = zeros(8,8);
% 
% BitmapMatrix(1,:) = ones(1, size(BitmapMatrix,2)); 
% 
% TxBlocksBitMap = reshape(BitmapMatrix, [1 L_Max]);
% 
% ssb.TransmittedBlocks = TxBlocksBitMap;
% 
% ssb.SubcarrierSpacingCommon = 120;


%% Set the Carrier Configuration

carrier = nrCarrierConfig;
carrier.SubcarrierSpacing = 120;
carrier.NSizeGrid = 32;

info = nrOFDMInfo(carrier)

%% Generate the SSB time domain signal

%[waveform,waveformInfo] = nrWaveformGenerator(Carrier);

%plot(real(waveform));hold on; plot(imag(waveform), "r")

%% Plot spectrogram

% Feed the recevied SSB demodulated block into that function to get the
% RSRP criteria value directly !
%meas = nrSSBMeasurements(ssbGrid,ncellid)

NCellID = 42;

nSubframes = 5;
symbolsPerSlot = 14;
mu = 3;

nSymbols = symbolsPerSlot * 2^mu * nSubframes;

ssburst = zeros([240 nSymbols]);

n = [0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18];

%n = n(1:2:end);

firstSymbolIndex = [4; 8; 16; 20] + 28*n;
%firstSymbolIndex = 4 + 28*n;
firstSymbolIndex = firstSymbolIndex(:).';

cw = randi([0 1],864,1);

ssblock = zeros([240 4]);
ssblock(nrPSSIndices) = nrPSS(NCellID);
ssblock(nrSSSIndices) = 2 * nrSSS(NCellID);

for ssbIndex = 1:length(firstSymbolIndex)
    
    i_SSB = mod(ssbIndex - 1,8);
    ibar_SSB = i_SSB;
    v = i_SSB;
    
    pbchSymbols = nrPBCH(cw,NCellID,v);
    ssblock(nrPBCHIndices(NCellID)) = 3 * pbchSymbols;
    
    dmrsSymbols = nrPBCHDMRS(NCellID,ibar_SSB);
    ssblock(nrPBCHDMRSIndices(NCellID)) = 4 * dmrsSymbols;
    
    ssburst(:,firstSymbolIndex(ssbIndex) + (0:3)) = ssblock;
    
end



imagesc(abs(ssburst));
clim([0 4]);
axis xy;
xlabel('OFDM symbol');
ylabel('Subcarrier');
title('SS burst, block pattern Case B');




%%

RB_Empty = ((carrier.NSizeGrid * 12) - 240) / 2;

Empty_Ressource = zeros(RB_Empty, size(ssburst,2));

grid = [Empty_Ressource; ssburst; Empty_Ressource];

f_0 = 27999840000; % Carrier frequency (relevant here for baseband signal ?), 0 by default...   

[waveform,info] = nrOFDMModulate(carrier,grid, 'Nfft', info.Nfft, 'SampleRate', info.SampleRate, 'CarrierFrequency', f_0);


figure;
plot((1:length(waveform))/info.SampleRate, real(waveform), "b");hold on; 
plot((1:length(waveform))/info.SampleRate, imag(waveform), "r");xlabel("Time [s]");title("SSB signal in time domain")
