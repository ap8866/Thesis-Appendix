% script to generate test cases for I and Q Data

%% Setup - model file
modelfile = '2400mS_Leopard.dat';
model = importdata(modelfile);

%% Setup - defining different segments, sampling frequency, etc. 

filename = 'testdata.dat'; % filename of file with data

freq = 10;   % sampling frequency
T = 1/freq;

samples(1,1) = 200; % number of samples in 1st segment
samples(1,2) = 15;  % height in 1st segment in m over water
samples(1,3) = 1;   % ice thickness in m
samples(2,1) = 200; % number of samples in 2nd segment
samples(2,2) = 20;  % height in 2nd segment in m over water
samples(2,3) = 1.5; % ice thickness in m
samples(3,1) = 200; % etc.
samples(3,2) = 15;
samples(3,3) = 2;   
samples(4,1) = 200;
samples(4,2) = 15;
samples(4,3) = 2.5;   

%% creating data

% Number of samples in total for all segments
N = 0; 
for i=1:length(samples(:,1))
    N = N+samples(i,1);
end
data = zeros(N,3);

startindex = 1; % initialization of index for writing data to data array
stopindex = 1;  % initialization of index for writing data to data array

for i=1:length(samples(:,1))
    EMheight = samples(i,2);
    % finding the closest value of the height value in the model 
    % ih is the index of the height array that has the closest value to "height"
    % d is the value
    [d,ih ] = min( abs(model.data(:,1)-EMheight) );

    % display information, if the exact value of heigth can't be found
    if d>0.001
        fprintf('height value entered:%f\n',EMheight);
        fprintf('height value chosen from model:%3.3f\n',model.data(ih,1));
    end
    clear d;
    
    if i==1
        startindex = 1;
        stopindex = samples(i,1);
    else
        startindex = stopindex +1; % one after last stopindex
        stopindex = startindex + samples(i,1) - 1;
    end
    
    if i == 3
        for j=startindex:stopindex
            data(startindex:stopindex,1) = model.data(ih,2)+(20*(rand(samples(i,1),1)-0.5));
            data(startindex:stopindex,2) = model.data(ih,3);
            data(startindex:stopindex,3) = model.data(ih,1)-samples(i,3);
        end
    elseif i==4
         for j=startindex:stopindex
            data(startindex:stopindex,1) = model.data(ih,2);
            data(startindex:stopindex,2) = model.data(ih,3)+(20*(rand(samples(i,1),1)-0.5));
            data(startindex:stopindex,3) = model.data(ih,1)-samples(i,3);
        end
        
    else
        data(startindex:stopindex,1) = model.data(ih,2);
        data(startindex:stopindex,2) = model.data(ih,3);
        data(startindex:stopindex,3) = model.data(ih,1)-samples(i,3);
        
    end
end

clear startindex stopindex T ih heigth i;

%% write data to file

fileID = fopen(filename,'w');

fprintf(fileID, '%5s %5s %10s\r\n','I','Q','height');
fprintf(fileID,'%4.3f %4.3f %3.3f\r\n',data');
fclose(fileID);

%% plot data

figure(1);
clf
plot(model.data(:,1),model.data(:,2));
hold on
plot(model.data(:,1),model.data(:,3),'r');
xlabel([string(model.textdata(1,1)) '/m'])
ylabel('PPM')
%legend('I','Q')
legend(string(model.textdata(1,2)),string(model.textdata(1,3)))
xlim([0 20])

figure(2)
clf
plot(data(:,1))
hold on
plot(data(:,2),'r')
plot(data(:,3),'k')
legend('I','Q','height')