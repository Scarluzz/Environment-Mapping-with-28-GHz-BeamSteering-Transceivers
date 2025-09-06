clear; 
clc; 
close all; 
set(0,'DefaultFigureWindowStyle', 'docked');

%% Primary parameters
directoryname = "/home/linuxasus/Bureau/MFE_Project/";

Exp_names = ["LoS_Setup/", "Moved_antenna_Setup/", "Obstacle_Reflection_Setup/"];

Exp_index = 3;

filename = directoryname + Exp_names(Exp_index) + "ExhaustiveFULL.dat"; 

%% Secundary parameters

fileId = fopen(filename, 'r');

%read parameters
rawData = textscan(fileId, 'Rate is %d and freq is %f', 1);
param.rate = rawData{1};
param.freq = rawData{2};

rawData = textscan(fileId, 'Gain tx is %d and Gain rx is %d', 1);
param.gain_tx = rawData{1};
param.gain_rx = rawData{2};

rawData = textscan(fileId, 'Number samps per degree is %d ', 1);
param.n_samps_per_degree = rawData{1};

rawData = textscan(fileId, 'Thetas min and max of tx and rx are %d %d %d %d', 1);
param.theta_min_tx = rawData{1};
param.theta_max_tx = rawData{2};
param.theta_min_rx = rawData{3};
param.theta_max_rx = rawData{4};

rawData = textscan(fileId, 'Step is %d and N active antennas are %d ', 1);
param.step = rawData{1};
param.n_active_antennas = rawData{2};

rawData = textscan(fileId, 'Length of active packet is %d', 1);

nbr_spls_packet     = rawData{1};

rawData = textscan(fileId, 'Over Sampling at Receiver %d %d', 1);

Oversampling = rawData{1};
Upsample_factor = rawData{2};

nbr_spls_beam       = param.n_samps_per_degree;

if (Oversampling)
    nbr_spls_beam = nbr_spls_beam * Upsample_factor;
end


tx_number_beams = (param.theta_max_tx - param.theta_min_tx)/param.step + 1; 
rx_number_beams = (param.theta_max_rx - param.theta_min_rx)/param.step + 1;

tx_angles = param.theta_min_tx:param.step:param.theta_max_tx;
rx_angles = param.theta_min_rx:param.step:param.theta_max_rx;


nbr_beams           = tx_number_beams*rx_number_beams;

corr_treshold       = 8;


M = nbr_spls_beam;
N = nbr_spls_packet;

clr = 'brgycmk';
cptClr = 0;

% load("Generate SSB/pilotSSB.mat");
% 
% pilots = zeros([tx_number_beams, N], "single");
% 
% for i = 1:tx_number_beams
%     front = (i-1)*10000 + 1;
%     pilots(i, :) = cleanSignal(front:front + N - 1);
% end

%% Generate transmitted packets

fprintf(sprintf('-- Generating Tx data packet ...\n'));

% Define the complex number
complex_number = (0.50000 + (0.50000)*1i);

% Initialize the tx_packet array
tx_packet = complex_number * ones(1, 1000);

for i = 251:500
    tx_packet(i) = -real(tx_packet(i)) + imag(tx_packet(i)) * 1i;
end

% Segment 3: 501 to 750 (Negate the imaginary part)
for i = 501:750
    tx_packet(i) = real(tx_packet(i)) - imag(tx_packet(i)) * 1i;
end

% Segment 4: 751 to 1000 (Negate both real and imaginary parts)
for i = 751:1000
    tx_packet(i) = -real(tx_packet(i)) - imag(tx_packet(i)) * 1i;
end
 % tx_packet;
 % tx_packet

tx_packet = generate_tx_packet();

%% Fetch Upsampled QPSK signal (by a certain factor)

% load("Generate SSB/QPSK_signal_upsampled.mat");
% 
% tx_packet = QPSK_signal_upsampled;
% 
% nbr_spls_packet = 1000*Upsample_factor;
% 
% N = nbr_spls_packet;
% 
% tx_packet = tx_packet(1:N);

%% Load data

fprintf(sprintf('-- Loading data ...\n'));
cptBeams= 0;


while (cptBeams < nbr_beams)

    % for k = 1:100
    cptBeams = cptBeams + 1;
  
    tx_index = ceil(cptBeams/double(rx_number_beams));
    rx_index =  mod(cptBeams-1, double(rx_number_beams)) + 1;

    %tx_packet = pilots(tx_index, :);

    fprintf(sprintf('  -- Tx angle = %f deg -- Rx angle = %f ...\n', tx_angles(tx_index), rx_angles(rx_index)))

    % Reading USRP data
    % ====cptBeams=============
    rawData = single(fread (fileId, 2*nbr_spls_beam, 'float'));
    %rawData = fread(fileId, 2*nbr_spls_beam, 'single');
    usrp_data = rawData(1:2:end) + rawData(2:2:end)*1i;

    % figure(888);plot(real(usrp_data));title("Combination number : " + ", TX : " + string(tx_angles(tx_index)) + ", RX : " + string(rx_angles(rx_index)))
    % figure(999);plot(real(usrp_data),imag(usrp_data),'*');title("CONSTELLATION. Combination number : " + string(k) + ", TX : " + string(tx_angles(tx_index)) + ", RX : " + string(rx_angles(rx_index)))

    % end


   %figure;     
   %plot(real(usrp_data(1:end)),imag(usrp_data(1:end)),'*')
   
    % if ((tx_angles(tx_index) >= -45 && tx_angles(tx_index) <= -30) && (rx_angles(rx_index) >= -45 && rx_angles(rx_index) <= -30))
    % Calculate normalized correlation 
    % ================================
    for cptSample = 1:M-N
        corr(cptSample) = usrp_data(cptSample:cptSample+N-1).' * (tx_packet'); 
        pow2(cptSample) = usrp_data(cptSample:cptSample+N-1).' * conj(usrp_data(cptSample:cptSample+N-1));
        corrNorm(cptSample) = corr(cptSample) ./ sqrt(pow2(cptSample));
    end
    % else
    %     corrNorm = zeros(1, M-N);
    % end
    %figure(101); hold on;
    cptClr = cptClr+1;
    if cptClr == 8
       cptClr = 1;
   
    end
    color = clr((cptClr));

    %plot(real(usrp_data));
    %figure(23);plot(abs(corrNorm));
    

    I1 = find (abs(corrNorm) > corr_treshold); 
    cptIndex = 0;
    indexStartPacket = [];
    for cpt = 1:length(I1)
        
        if ( (abs(corrNorm(I1(cpt))) > abs(corrNorm(I1(cpt)-1))) && ...
                (abs(corrNorm(I1(cpt))) > abs(corrNorm(I1(cpt)+1))) )
            cptIndex = cptIndex+1;
            indexStartPacket(cptIndex) = I1(cpt);
        end
    end 

%% From correlation peaks, get the true complex symbols

    realpacketlength = nbr_spls_packet/Upsample_factor;
    nbr_of_packet_per_burst = 10;
    truesymbols = zeros([1 realpacketlength*nbr_of_packet_per_burst], "single");

    for j = 1:length(indexStartPacket)
        truesymbols((j-1)*realpacketlength+1:j*realpacketlength) = usrp_data(indexStartPacket(j):Upsample_factor:indexStartPacket(j)+ (realpacketlength-1)*Upsample_factor);
    end
    
    %figure(999);plot(real(truesymbols(1:realpacketlength)),imag(truesymbols(1:realpacketlength)),'*');title("CONSTELLATION. Combination number : " + string(cptBeams) + ", TX : " + string(tx_angles(tx_index)) + ", RX : " + string(rx_angles(rx_index)))


    % Calculating RSS of packets
    % ==========================

  if (~isempty(indexStartPacket))
        %% 
        rss = zeros(1,length(indexStartPacket));
        
        if(~Oversampling)
            for cptIndex = 1:length(indexStartPacket)
                rss(1, cptIndex) = mean(abs(usrp_data(indexStartPacket(cptIndex):indexStartPacket(cptIndex)+nbr_spls_packet-1)));
            end
            mean_rss(cptBeams) = mean(rss);
        else
            mean_rss(cptBeams) = mean(abs(truesymbols(1:realpacketlength)));
        end
        
    else
        mean_rss(cptBeams) = 0;
    end
  
    clear corr pow2 corrNorm; 
    clear I1 usrp_data indexStartPacket rss;  


end


%tx_angles = tx_angle(1:rx_number_beams:end);
%rx_angles = rx_angle(1:rx_number_beams);

Need_to_check_next_Line = "Need_to_check_next_Line!"

mean_rss_txrx = reshape(mean_rss, rx_number_beams, tx_number_beams);

figure; hold on; 
pl = surf(tx_angles, rx_angles, mean_rss_txrx);
set(pl, 'EdgeAlpha', 0)
colorbar; 
xlabel('AoD (deg)'); ylabel('AoA (deg)'); 

figure; hold on; grid on; 
%pl = surf(tx_angles, rx_angles, 20*log10(mean_rss_txrx));

%mean_rss_txrx(mean_rss_txrx == 0) = min(min(mean_rss_txrx(mean_rss_txrx > 0))) * 0.1; % Avoid log of zero

pl = imagesc(tx_angles, rx_angles, 20*log10(mean_rss_txrx));
%set(pl, 'EdgeAlpha', 0)
colorbar; axis equal; 
xlabel('AoD (deg)'); ylabel('AoA (deg)'); 
% 
% plot(tx_angles(column), rx_angles(row), '.');
% xlim([-45 45])

%% Get max of exhaustive search found Tx/Rx combination

[maxval, ind] = max(mean_rss);

maxvaldB = 20*log10(maxval);

tx_index_max = ceil(ind/double(rx_number_beams));
rx_index_max =  mod(ind-1, double(rx_number_beams)) + 1;

fprintf(sprintf('Max is found for Tx angle = %f deg & Rx angle = %f \n And is valued at : %d =  %d dB', tx_angles(tx_index_max), rx_angles(rx_index_max), maxval, maxvaldB))


% figure;
% 
% mean_rss_txrx_dB = 20*log10(mean_rss_txrx);
% 
% only_number = mean_rss_txrx_dB(mean_rss_txrx_dB ~= -Inf);
% 
% mean_rss_txrx_dB(mean_rss_txrx_dB == -Inf) = min(only_number);
% 
% pl = imagesc(tx_angles, rx_angles, mean_rss_txrx_dB);
% 
% 
% 
% 
% %set(pl, 'EdgeAlpha', 0)
colorbar; axis equal; 
xlabel('AoD (deg)'); ylabel('AoA (deg)'); 

title("Heatmap : Measured RSS in dB on the angular space");subtitle("Per AoD/AoA combination")
