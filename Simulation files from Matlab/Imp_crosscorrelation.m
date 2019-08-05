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
% --- configureA the simuilation ----
% ----------------------------------

ftx  = 5585;            % Tx Frequency, corresponds to Bird frequency
ATx = 1.5;                % Amplitude of Tx Signal, digital value
ARx = 1*ATx;         % Amplitude of Rx Signal, digital value
                        % so far, no direct path from Tx to Rx (perfect
                        % bucking)

noiseTx = 0.0*ATx ; % amplitude of noise in Tx signal
noiseRx = 0.0*ARx ; % amplitude of noise in Rx signal

RxPhase = 20;       % Phase of Rx signal in degree in relation to Tx.
                    % Tx signal starts with a random phase.

Aref = 1;           % amplitude of reference Signal for I-Q Demodulation
fref = 5585;        % frequency of reference signal

fs = 48000;         % samling rate in samples per second 4800000
N = 128;           % Number of samples, corresponds to sample length 12800
                    % sample time: (N-i1)/fs ~= N/fs

% ------------------------------
% ---- end of configuration ----
% ------------------------------

% ---- time vector ----
t = 0:(1/fs):(N-1)/fs;  % N values with the distance of Ts=1/fs


% ---- Signals to calculate I and Q from ----
%initphase = 2*pi*rand(1);               % initial phase of Tx Signal, random
initphase = 2*pi*0;               % initial phase of Tx Signal, random
TxPhase = initphase;                    % Tx starting phase
RxPhase = initphase+((pi/180)*RxPhase); % Phase of Rx Signal in relation to Tx

Stx = ATx*cos(2*pi*ftx*t+TxPhase) + noiseTx*rand(size(t)); % Signal 1
Srx = ARx*cos(2*pi*ftx*t+RxPhase) + noiseRx*rand(size(t)); % Signal 2

[cm, lag] = xcorr(Stx,Srx,'coeff'); % crosscorelation command in matlab
[~,I] = max(abs(cm));
%maxabs= max(abs(cm));
lagDiff = lag(I);
timeDiff = lagDiff/fs;
phi = 360 * fref * timeDiff;
disp(phi)
% --- Plots -----
figure(1)
clf
subplot(2,1,1)
plot(t,Stx,'LineWidth',1.5)
hold on 
grid on
plot(t,Srx,'LineWidth',1.5)
l = legend('Modulated signal 1','Modulated signal 2');
l.FontSize = 15;
title('(a) Input signals','FontName','Times New Roman','FontSize',24)
xlabel('Time in sec','FontName','Times New Roman','FontSize',24)
ylabel('Amplitude, digital values','FontName','Times New Roman','FontSize',24)

subplot(2,1,2)
%plot(t,dotIrx)
hold on
grid on
box on
plot(lag,cm,'LineWidth',1.5);
% plot(t,dotItx)
% plot(t,dotQtx)
l = legend('Crosscorrelation');
l.FontSize = 15;

title('(b) Crosscorrelation of two modulated signal','FontName','Times New Roman','FontSize',24)
xlabel('Lag -(N-1) to (N-1)','FontName','Times New Roman','FontSize',24)
ylabel('Cross-correlation','FontName','Times New Roman','FontSize',24)

% subplot(3,1,3)
% plot(f,abs(specIrx),'r','LineStyle','-')
% hold on
% grid on
% plot(f,abs(specQrx),'k','LineStyle','-')
% % plot(f,abs(specItx),'r','LineStyle','--')
% % plot(f,abs(specQtx),'k','LineStyle','--')
% legend('Spectrum of I','Spectrum of Q')
% title(['spectrum of dot product of Rx signal with I- and Q-component' ...
%     'of reference signal (not used to calculate I and Q)'])
% xlabel('frequency in Hz')
% ylabel('Amplitude, digital values')

