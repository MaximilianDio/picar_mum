%% speed data
clear
close all
clc

%%
% speed corresponds to value given in ROS

%distanz zwischen Lichtschranken = 1m
deltaL = 1; %m

filenames = ["tek0000All.csv";
             "tek0001All.csv";
             "tek0002All.csv";
             "tek0003All.csv";
             "tek0004All.csv";
             "tek0005All.csv";
             "tek0006All.csv";
             "tek0007All.csv";
             "tek0008All.csv"];
         
NUM_ELEM = length(filenames);
speed = zeros(NUM_ELEM,1); 
for ii = 1:NUM_ELEM
    deltaTime02 = get_deltaTime(filenames(ii));
    
    speed(ii,1) = deltaL/deltaTime02; %m/sec
end




function deltaTime = get_deltaTime(filename)
    [time, speed_ch1, speed_ch2]  = get_speedData(filename);
    
    %%replace nan with 0
    time(isnan(time))=0;
    speed_ch1(isnan(speed_ch1)) = 0.24;
    speed_ch2(isnan(speed_ch2)) = 0.24;
    
    speed_avarage = mean(speed_ch1(1:end-1)); %% get avarage of speed

    idx_ch1 = find(  speed_ch1 > speed_avarage);
    idx_ch1 = idx_ch1(1);
    time_ch1 = time(idx_ch1);

    idx_ch2 = find(  speed_ch2 > speed_avarage);
    idx_ch2 = idx_ch2(1);
    time_ch2 = time(idx_ch2);

    deltaTime = time_ch2 - time_ch1;


    plot(time(1:end-1),[speed_ch1(1:end-1) , speed_ch2(1:end-1)] )
end
