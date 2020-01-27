close all
clear all
clc
%%  Plot Tracked Sine Waves
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%GetData
[t,vel_des, vel_est] = getTrackedSine('tracking_sine_better.txt');

t = t-t(1)-5;

%Plot
plot(t,vel_des,'b','LineWidth', 1.5);
hold on ;
plot(t,vel_est,'r','LineWidth', 1.5);
legend('vorgeschriebene Geschwindigkeit','gesch\"atzte Geschwindigkeit','Location','southeast','FontSize',12,'Interpreter','latex');
grid on
xlim([0 15])
ylim([0 1.2])
xlabel("Zeit [s]",'Interpreter','latex');
ylabel("Geschwindigkeit [m/s]",'Interpreter','latex');
dim = [.15 .5 .3 .3];

set(gca,'FontSize',12);
set(gcf, 'Position', [100, 100, 650, 400]);