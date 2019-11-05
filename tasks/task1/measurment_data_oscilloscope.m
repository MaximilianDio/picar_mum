%% oscilloscope data
clear
close all
clc


%%
[time, values_without]  = get_oscillatordata("meas1_no_car.csv");
[time, values_with]  = get_oscillatordata("meas1_with_car.csv");

plot(time(1:end-1),values_without(1:end-1))
hold on
plot(time(1:end-1),values_with(1:end-1))



