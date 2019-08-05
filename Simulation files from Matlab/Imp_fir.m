% Program for testing Algorithms to calculate I and Q.
% In this example the I and Q components of two input signals (Tx and Rx)
% is calculated. 
% The two input signals are each multiplied with a sine and cosine signal,
% the dot product of the signals. The for resulting signals are lowpass
% filtered to extract the DC-Part of the dot product. Afterwards the mean
% value of the filtered signals is calculated, which then represents the I
% and Q data for the two input signals. From those four values (I and Q
% from Tx and Rx) the phase difference between the two signals can be
% calculated.  

clear

% ----------------------------------
% --- configure the simuilation ----
% ----------------------------------

ftx  = 5585;            % Tx Frequency, corresponds to Bird frequency
ATx = 1.5;                % Amplitude of Tx Signal, digital value
ARx = 0.25*ATx;         % Amplitude of Rx Signal, digital value
                        % so far, no direct path from Tx to Rx (perfect
                        % bucking)

noiseTx = 0.0*ATx ; % amplitude of noise in Tx signal
noiseRx = 0.0*ARx ; % amplitude of noise in Rx signal

RxPhase = 20;       % Phase of Rx signal in degree in relation to Tx.
                    % Tx signal starts with a random phase.

Aref = 1;           % amplitude of reference Signal for I-Q Demodulation
fref = 5585;        % frequency of reference signal

fs = 48000;         % samling rate in samples per second
N = 128;           % Number of samples, corresponds to sample length
                    % sample time: (N-i1)/fs ~= N/fs

% ------------------------------
% ---- end of configuration ----
% ------------------------------

% ---- time vector ----
t = 0:(1/fs):(N-1)/fs;  % N values with the distance of Ts=1/fs


% ---- Signals to calculate I and Q from ----
initphase = 2*pi*rand(1);               % initial phase of Tx Signal, random

TxPhase = initphase;                    % Tx starting phase
RxPhase = initphase+((pi/180)*RxPhase); % Phase of Rx Signal in relation to Tx

Stx = ATx*cos(2*pi*ftx*t+TxPhase) + noiseTx*rand(size(t)); % Signal 1
Srx = ARx*cos(2*pi*ftx*t+RxPhase) + noiseRx*rand(size(t)); % Signal 2

% ---- Reference Signal -----
coswave = Aref*cos(2*pi*fref*t);
sinwave = Aref*sin(2*pi*fref*t);

% ---- muiltiplication with reference signal ----
dotItx = coswave.*Stx; % dot product of Tx signal with I component of reference signal
dotQtx = sinwave.*Stx; % dot product of Tx signal with Q component of reference signal

dotIrx = coswave.*Srx; % dot product of Rx signal with I component of reference signsl 
dotQrx = sinwave.*Srx; % dot product of Rx sgianl with Q component of reference signal

% --- spectrum of mixed signals ----

f = ((-N/2):(N/2-1))*(fs/N); % frequency vector
specIrx = fft(dotIrx)./length(dotIrx);  % FFT of dot product of Rx Signal 
                                        % and I component of reference signal
specIrx = fftshift(specIrx);            % shifting 0Hz to the middle of plot

specQrx = fft(dotQrx)./length(dotQrx);  % FFT of dot product of Rx Signal 
                                        % and Q component of reference signal
specQrx = fftshift(specQrx);            % shifting 0Hz to middle of plot

% spectrum of dotproduct of Tx signal not calculated to make 
% plot not too messy

% ---- lowpassfiltering I- and Q-Data with FIR filter -----

n = 30;             % FIR filter order

%Wn = 0.5; 
Wn = (50*2)/fs;      % cut-off frequency 0 < Wn < 1, 1 corresponds to fs/2
B = fir1(n,Wn) ;     % calculating filter coefficients 

% figure(2)         %plotting frequency and phase respons of filter
% clf
% freqz(B)

dotItxlp = filter(B,1,dotItx);  % filter dot products
dotQtxlp = filter(B,1,dotQtx);  % filter dot products

dotIrxlp = filter(B,1,dotIrx);  % filter dot products
dotQrxlp = filter(B,1,dotQrx);  % filter dot products

% ---- calculate I and Q through averaging -----
Itx = mean(dotItxlp(n+1:length(dotItxlp-1))); % Inphase value of Tx signal
                                   % discard first n+1 values because 
                                   % of the transient of the LP-Filter.
                                   % Multiply by 2 to compensate for 
                                   % half the energy that was in the 
                                   % high frequency part of the dot
                                   % product (see the math for details).                             
                                 
Qtx = mean(dotQtxlp(n+1:length(dotQtxlp-1))); % Quadrature value of Tx signal 

Irx = mean(dotIrxlp(n+1:length(dotIrxlp-1))); % Inphase of Rx Signal
Qrx = mean(dotQrxlp(n+1:length(dotQrxlp-1))); % Quadrature of Rx Signal

fprintf('Itx = %f\n',Itx)
fprintf('Qtx = %f\n\n',Qtx)

fprintf('Irx = %f\n',Irx)
fprintf('Qrx = %f\n',Qrx)

% ---- calculate Amplitude of Tx and Rx -----

ATxdemod = 2* sqrt((Itx^2)+(Qtx^2)); % calculated amplitude of Tx signal
ARxdemod = 2*sqrt((Irx^2)+(Qrx^2)); % calculated amplitude of Rx signal

% ---- calculate Phase of Tx and Rx -----
% phase is calculated in degress from Inphase axis counterclockwise 
% and only positive. When the phase of Rx and Tx is calculated
% the quadrant is checked and the result of the atan() "corrected" to
% a positive value. For details check definition of arctan.

phitx = 0;  % initialize variable
phirx = 0;  % initialize variable
    
% Tx Phase:
if((Itx>=0) && (Qtx>=0))        % first Quadrant
   phitx = atan(Qtx/Itx);       
elseif((Itx<0) && (Qtx>=0))     %second quadrant
    phitx = atan(Qtx/Itx) + pi; 
elseif((Itx<0) && (Qtx<0))      % third qudrant
    phitx = atan(Qtx/Itx) + pi;
elseif((Itx>=0) && (Qtx<0))     % fourth quadrant
    phitx = atan(Qtx/Itx) + (2*pi);
else
    fprintf('something wrong with calculating the tx phase')
end

% Rx Phase:
if((Irx>=0) && (Qrx>=0))        % first Quadrant
   phirx = atan(Qrx/Irx);  
elseif((Irx<0) && (Qrx>=0))     %second quadrant
    phirx = atan(Qrx/Irx) + pi;
elseif((Irx<0) && (Qrx<0))      % third qudrant
    phirx = atan(Qrx/Irx) + pi;
elseif((Irx>=0) && (Qrx<0))     % fourth quadrant
    phirx = atan(Qrx/Irx) + (2*pi);
else
    fprintf('something wrong with calculating the rx phase')
end

phitx = phitx*(180/pi); % converting rad in degree
phirx = phirx*(180/pi); % converting rad in degree

% ---- Phasedifference between Rx and Tx ----
% Rx is assumed to always lag behin Tx. If phase angle of Rx is greater
% than angle of the Tx signal, than Tx Signal is assumed to crossed the
% Inphase axis from the fourth quadrant and 360degree is added to Tx phase. 

phidiff = 0; % initialize variable

if(phitx>=phirx)           % Tx Phase is greater than Rx Phase
    phidiff = phitx-phirx;
elseif(phitx<phirx)        % Tx Phase smaller than Rx phase 
    phidiff = 360+phitx-phirx;
else
    fprintf('ERROR calculating phasedifference')
end

fprintf('\nTx Phase: %f\n',phitx)
fprintf('Rx Phase: %f\n',phirx)
fprintf('Rx is signal is %f degrees behin Tx\n',phidiff)

% --- Plots -----
figure(1)
clf
subplot(3,1,1)
plot(t,Stx,'LineWidth',1.5)
hold on 
grid on
plot(t,sinwave,'LineWidth',1.5)
plot(t,coswave,'r','LineWidth',1.5)
l = legend('Modulated signal','Reference sine', 'Reference cosine');
l.FontSize = 15;
title('(a) Input and Reference signals','FontName','Times New Roman','FontSize',24)
xlabel('Time (seconds)','FontName','Times New Roman','FontSize',24)
ylabel('Amplitude (Volts)','FontName','Times New Roman','FontSize',24)

subplot(3,1,2)
plot(t,dotIrx,'LineWidth',1.5)
hold on
grid on
plot(t,dotQrx,'r','LineWidth',1.5)
l = legend('Dot I','Dot Q');
l.FontSize = 15;
title('(b) Dot product of the modulated signal with reference signals'...
    ,'FontName','Times New Roman','FontSize',24)
xlabel('Time (seconds)','FontName','Times New Roman','FontSize',24)
ylabel('Amplitude (Volts)','FontName','Times New Roman','FontSize',24)

subplot(3,1,3)
plot(t,dotIrxlp,'LineWidth',1.5)
hold on
grid on
plot(t,dotQrxlp,'LineWidth',1.5)
title('(c) dot I and dot Q after lowpass filter','FontName','Times New Roman','FontSize',24)
l = legend('dot I lowpass','dot Q lowpass');
l.FontSize = 15;
xlabel('Time (seconds)','FontName','Times New Roman','FontSize',24)
ylabel('Amplitude (Volts)','FontName','Times New Roman','FontSize',24)

