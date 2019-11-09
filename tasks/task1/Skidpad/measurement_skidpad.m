clear all
clc
%% import data and run

filenames = ["vel=0,3_ang_real=14,89_stear=16,5.txt";
             "vel=0,6_ang_real=14,89_stear=17,75.txt";
             "vel=0,9_ang_real=14,89_stear=18,5.txt"];

accel_data=length(filenames);
angle_data=length(filenames);
accel_std=length(filenames);

% fudge factor
fpass=double(8); %cutoff lowpass

%compute accels

   for n=1:length(filenames)
    [time,yaw,accel_x,accel_y]=get_imu_data(filenames(n));%import data
    time=time-time(1); %correct time
    fs=1/(time(2)-time(1)); %sampling freq
    
    [accel_data(n),accel_std(n)] = get_accels(accel_x,accel_y,fpass,fs)
   end


%compute angles
angles_unreal=[16.5;17.75;18.5];
   for n=1:length(angles_unreal)
   angle_data(n)=G_angles(angles_unreal(n));
   end
   


%% plot results
plot(accel_data,angle_data)
hold on;
%Daten","Quadratische Interpolation");

xlabel("Querbeschleunigung [m/s^2]");
ylabel("Lenkradwinkel [?]")

%% functions

%% Main
function [accel_mean,accel_std] = get_accels(accel_x,accel_y,fpass,fs)
%filtering of fluctuations
x=lowpass(accel_x,fpass,fs);
y=lowpass(accel_y,fpass,fs);

%computation of lateral acceleration
%lateral=sqrt(x.^2+y.^2);
lateral=y

%Double Check of fft of lateral
L=length(lateral);
f=fs*(0:(L/2))/L;
Y=fft(lateral);
P2 = abs(Y);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

%spectrogram(P1);

%select data for mean compution
threshold=0.006
index=find(abs(lateral)>=threshold);

upperbound=round(length(index)*1/4);
lowerbound=round(length(index)*3/4);

a=index(upperbound,1);
b=index(lowerbound,1);

%Computation of accel
accel_mean=mean(lateral(a:b,1));
accel_std=std(lateral(a:b,1));
end

%% Quadratic Interpolation
function y = Q_accels(x)
   
    
    % Coefficients:
      p1 = 0.87857;
      p2 = 0.25;
      %p3=

    y = p1*x + p2;
end


%% transformation stearing angle
function y = G_angles(x)
    angleDesired = 0;%14.95; %in deg
    
    % Coefficients:
      p1 = 0.87857;
      p2 = 0.25;

    y = p1*x + p2 - angleDesired; 
end

