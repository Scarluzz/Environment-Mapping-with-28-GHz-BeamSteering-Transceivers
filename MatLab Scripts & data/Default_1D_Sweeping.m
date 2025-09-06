clear; 
clc; 
close all; 
set(0,'DefaultFigureWindowStyle', 'docked');


%% Primary parameters
directoryname = "/home/linuxasus/Bureau/MFE_Project/";
filename            = directoryname + 'ResultsExp.dat'; 

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


nbr_spls_beam       = param.n_samps_per_degree;

%nbr_spls_beam = 50000 au0;

rawData = textscan(fileId, 'Length of active packet is %d', 1);

%nbr_spls_packet     = rawData{1};

nbr_spls_packet = 1000;

tx_number_beams = (param.theta_max_tx - param.theta_min_tx)/param.step + 1; 
rx_number_beams = (param.theta_max_rx - param.theta_min_rx)/param.step + 1;

nbr_beams           = tx_number_beams*rx_number_beams;

corr_treshold       = 10;


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


%% Load data

fprintf(sprintf('-- Loading data ...\n'));
cptBeams= 0;


while (cptBeams < nbr_beams)

    for k = 1:100
    cptBeams = cptBeams + 1;
    
    %tx_packet = pilots(ceil(cptBeams/double(rx_number_beams)), :);

    % Reading Tx data
    % ===================
    dataTycptBeamspe =  textscan(fileId,'%s',3);
    rawData = textscan(fileId, '%d at time %f', 1);
    tx_angle(cptBeams) = rawData{1};
    clear dataType rawData


    % Reading AiP Rx data
    % ===================
    dataTycptBeamspe =  textscan(fileId,'%s',3) ;
    rawData = textscan(fileId, '%d at time %f', 1);
    rx_angle(cptBeams) = rawData{1};
    clear dataType rawData
    
    fprintf(sprintf('  -- Tx angle = %f deg -- Rx angle = %f ...\n', tx_angle(cptBeams), rx_angle(cptBeams)))

    % Reading USRP data
    % ====cptBeams=============
    dataType = textscan(fileId,'%s',2);
    rawData = fread(fileId,1,'int8'); % Read end-of-line character
    rawData = single(fread (fileId, 2*nbr_spls_beam, 'float'));
    usrp_data = rawData(1:2:end) + rawData(2:2:end)*1i;
    
    rawData = fread(fileId,1,'int8'); % Read end-of-line character
    
    %figure(8);plot(real(usrp_data));title("Combination number : " + string(k) + ", TX : " + string(tx_angle(cptBeams)) + ", RX : " + string(rx_angle(cptBeams)))
    % figure(8);plot(real(usrp_data), "b");hold on; plot(imag(usrp_data), "r");
    % title("Measured SIgnal at Receiver - Beam Pair ID = " + string(k))
    % subtitle("Tx direction = -45°, Rx direction = -30°")
    % ylabel("Amplitude"); xlabel("samples");
    % legend("Real", "Imaginary");hold off;

    plot(real(usrp_data(1:end)),imag(usrp_data(1:end)),'*')

    end


   %figure;     
   %figure(1223);plot(real(usrp_data(1:end)),imag(usrp_data(1:end)),'*')
   

    % Calculate normalized correlation 
    % ================================
    for cptSample = 1:M-N
        corr(cptSample) = usrp_data(cptSample:cptSample+N-1).' * (tx_packet'); 
        pow2(cptSample) = usrp_data(cptSample:cptSample+N-1).' * conj(usrp_data(cptSample:cptSample+N-1));
        corrNorm(cptSample) = corr(cptSample) ./ sqrt(pow2(cptSample));
    end
    
    %figure(101); hold on;
    cptClr = cptClr+1;
    if cptClr == 8
       cptClr = 1;
   
    end
    color = clr((cptClr));

    %plot(real(usrp_data));
    %figure(4985);plot(abs(corrNorm));
    

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



    % Calculating RSS of packets
    % ==========================
  if (~isempty(indexStartPacket))
        %% 
        rss = zeros(1,length(indexStartPacket));
        for cptIndex = 1:length(indexStartPacket)
            rss(1, cptIndex) = mean(abs(usrp_data(indexStartPacket(cptIndex):indexStartPacket(cptIndex)+nbr_spls_packet-1)));
        end
        mean_rss(cptBeams) = mean(rss);
    else
        mean_rss(cptBeams) = 0;
    end
  
    clear corr pow2 corrNorm; 
    clear I1 usrp_data indexStartPacket rss;  


end


tx_angles = tx_angle(1:rx_number_beams:end);
rx_angles = rx_angle(1:rx_number_beams);
mean_rss_txrx = reshape(mean_rss, tx_number_beams, rx_number_beams);


figure; hold on; 
pl = surf(tx_angles, rx_angles, mean_rss_txrx);
set(pl, 'EdgeAlpha', 0)
colorbar; 
xlabel('AoD (deg)'); ylabel('AoA (deg)'); 

figure; hold on; grid on; 
%pl = surf(tx_angles, rx_angles, 20*log10(mean_rss_txrx));

%mean_rss_txrx(mean_rss_txrx == 0) = min(min(mean_rss_txrx(mean_rss_txrx > 0))) * 0.1; % Avoid log of zero

pl = surf(tx_angles, rx_angles, 20*log10(mean_rss_txrx));
set(pl, 'EdgeAlpha', 0)
colorbar; axis equal; 
xlabel('AoD (deg)'); ylabel('AoA (deg)'); 
% 
% plot(tx_angles(column), rx_angles(row), '.');
% xlim([-45 45])
