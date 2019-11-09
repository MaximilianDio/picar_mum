%% oscilloscope data
clear
close all
clc


%%
[time, values_without]  = get_oscillatordata("meas1_no_car.csv");
[time_new, values_with]  = get_oscillatordata("Meas_car_new.csv");

plot(time(1:3000)+10,values_without(1:3000), 'b')
hold on
plot(time_new(1:3000)+10,values_with(1:3000), 'r')
grid on
legend('SB', 'SB+C')
xlabel('time t in [s]')
ylabel('signal')



