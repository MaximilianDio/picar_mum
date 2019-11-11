%% oscilloscope data
clear
close all
clc


%%
[time, values_without]  = get_oscillatordata("meas1_no_car.csv");
[time_new, values_with]  = get_oscillatordata("Meas_car_new.csv");

plot(time(1:3000)+10,values_without(1:3000),'LineWidth', 1.5)
hold on
plot(time_new(1:3000)+10,values_with(1:3000),'LineWidth', 1.5)
grid on
set(gca,'FontSize',12)
set(gcf, 'Position', [100, 100, 650, 400]);
legend('SB', 'SB+C','Location','southeast','FontSize',12)
xlabel('Zeit [s]')
ylabel('|F(s)|')
ylim([0,12])



