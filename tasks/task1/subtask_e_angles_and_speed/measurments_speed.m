%% speed data
clear
close all
clc

%%
% speed corresponds to value given in ROS

%distanz zwischen Lichtschranken = 1m
deltaL = 1; %mhttps://help.github.com/en/github/setting-up-and-managing-your-github-user-account/inviting-collaborators-to-a-personal-repository

filenames = ["tek0000ALL.csv";
             "tek0001ALL.csv";
             "tek0002ALL.csv";
             "tek0003ALL.csv";
             "tek0004ALL.csv";
             "tek0005ALL.csv";
             "tek0006ALL.csv";
             "tek0007ALL.csv";
             "tek0008ALL.csv"];
         
NUM_ELEM = length(filenames);
speedOut = zeros(NUM_ELEM,1); 
for ii = 1:NUM_ELEM
    deltaTime = get_deltaTime(filenames(ii));
    
    speedOut(ii,1) = deltaL/deltaTime; %m/sec
end

speedIn = 0.2:0.1:1;

plot(speedIn,speedOut)
hold on;
plot(speedIn,G_speed(speedIn))
legend("measured Data","quadratic fitting curve");
% 
% xlabel("angles Input in Deg");
% ylabel("angles Output in Deg")
xlabel('Speed input')
ylabel('Velocity of Picar04 [m/s]')
%% quadratic interpolation
function y = G_speed(x)

    % Coefficients:
      p1 = -0.76149;
      p2 = 2.6308;
      p3 = -0.3301;


    y = p1*x.^2 + p2*x + p3; 

end


%% calculate time difference between two light barriers

function deltaTime = get_deltaTime(filename)
    [time, speed_ch1, speed_ch2]  = get_speedData(filename);
    
    %%replace nan with 0
    time(isnan(time))=0;
    speed_ch1(isnan(speed_ch1)) = 0.24;
    speed_ch2(isnan(speed_ch2)) = 0.24;
    
    speed_avarage = 6; %% get avarage of speed

    idx_ch1 = find(  speed_ch1 > speed_avarage);
    idx_ch1 = idx_ch1(1);
    time_ch1 = time(idx_ch1);

    idx_ch2 = find(  speed_ch2 > speed_avarage);
    idx_ch2 = idx_ch2(1);
    time_ch2 = time(idx_ch2);

    deltaTime = time_ch2 - time_ch1;
    
    plot(time(1:end-1),[speed_ch1(1:end-1) , speed_ch2(1:end-1)] )
end
