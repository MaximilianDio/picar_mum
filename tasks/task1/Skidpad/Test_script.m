clc
close all
clear all

%% import
radius = 1; %m

filenames = ["vel=0,3_ang_real=14,89_stear=16,5.txt";
             "vel=0,6_ang_real=14,89_stear=17,75.txt";
             "vel=0,9_ang_real=14,89_stear=18,5.txt"];

[time,yaw,accel_x,accel_y]=get_imu_data('vel=0,9_ang_real=14,89_stear=18,5.txt');%import data


L=length(accel_y)
time=time-time(1); %correct time
fs=1/(time(2)-time(1)); %sampling freq
f=fs*(0:(L/2))/L
accel_lat=sqrt(accel_x.^2+accel_y.^2)
spectrogram(accel_y,256,250,256,fs,'yaxis')
caxis([-80 -8])
ylabel('Frequenz [Hz]');
xlabel('Zeit [s]');
set(gca,'FontSize',11)
ylabel(colorbar,'Energie/Frequenz [dB/Hz]','FontSize', 11)
%% 
figure(1)
plot(time,accel_lat,time,accel_x,time,accel_y)
legend()

figure(2)
plot(f,compute_fft(accel_y),f,compute_filtered_fft(accel_y,15,fs))
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

figure(3)
plot(time,lowpass(accel_x,15,fs));

%% plot result
figure(1)
plot(accels,angles);
hold on;
accels_kont=[accels(1):accels(1)/100:accels(3)]
plot(accels_kont,Q_accels(accels_kont));
legend("measured Data","linear fitting curve");

xlabel("angles Input in Deg");
ylabel("angles Output in Deg")

%% compute fft
function P1=compute_fft(accel)
    L=length(accel);
    Y=fft(accel);
    P2 = abs(Y);
    P1 = P2(1:L/2+1);
 P1(2:end-1) = 2*P1(2:end-1);
end

%% Filter
function [P1,x]=compute_filtered_fft(accel,fpass,fs)
x = lowpass(accel,fpass,fs);
L=length(x);
Y=fft(x);
P2 = abs(Y);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
end

%% Quadratic Interpolation
function y = Q_accels(x)
   
    
    % Coefficients:
      p1 = 0.87857;
      p2 = 0.25;
      %p3=

    y = p1*x + p2;
end








