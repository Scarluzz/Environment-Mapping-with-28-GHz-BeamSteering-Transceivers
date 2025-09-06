clear; 
clc; 
close all; 
set(0,'DefaultFigureWindowStyle', 'docked');


%% Primary parameters
filename            = 'ResultsExp_good_buffer.dat'; 

%% Secundary parameters

fileId = fopen(filename, 'r');


nbr_spls_beam       = 100000;

%nbr_spls_beam = 50000 au0;
nbr_spls_packet     = 1000;
tx_number_beams = 10;
rx_number_beams = 10;

nbr_beams           = tx_number_beams*rx_number_beams;

corr_treshold       = 12; 


M = nbr_spls_beam; 
N = nbr_spls_packet; 

clr = 'brgycmk';
cptClr = 0;

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
    
    cptBeams = cptBeams + 1;
    %% 
    
for j = 1:100
%% 
    % Reading USRP data
    % ====cptBeams=============
    rawData = fread (fileId, 2*nbr_spls_beam, 'float');
    usrp_data= rawData(1:2:end) + rawData(2:2:end)*1i;
    %figure;     
    %plot(real(usrp_data(1:end)),imag(usrp_data(1:end)),'*');title(string(j))

    if j == 10
        mid = 45;
        figure(1);plot(real(usrp_data));title(string(j))
        figure(2);plot(real(usrp_data(1:end)),imag(usrp_data(1:end)),'*');title(string(j))
        %plot(abs(usrp_data));
    end

    figure(1);plot(real(usrp_data));title(string(j))
    %figure(2);plot(real(usrp_data(1:end)),imag(usrp_data(1:end)),'*');title(string(j))


end
"done through the file !"
end