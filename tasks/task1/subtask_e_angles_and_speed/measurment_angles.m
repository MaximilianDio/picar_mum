%%
clear
close all
clc
%%
anglesIn = 0:5:30;
anglesOut = [0;5;9;13;18;23;26];

plot(anglesIn,anglesOut,'LineWidth', 1.5,'Marker','x');
hold on;
plot(anglesIn,G_angles(anglesIn),'LineWidth', 1.5);
legend("Daten","Lineare Interpolation",'Location','southeast','FontSize',12);
grid on
xlabel("Eingangswinkel [?]");
ylabel("Ausgangswinkel [?]")
dim = [.15 .5 .3 .3];
str = 'y = 0.87857x + 0.25';
annotation('textbox',dim,'String',str,'FitBoxToText','on','Color',[0.8500, 0.3250, 0.0980],'FontSize',12);
set(gca,'FontSize',12)
set(gcf, 'Position', [100, 100, 650, 400]);

% angeDesiredIn = fsolve(@G_angles,15);

%% linear interpolation
function y = G_angles(x)
    angleDesired = 0;%14.95; %in deg
    
    % Coefficients:
      p1 = 0.87857;
      p2 = 0.25;

    y = p1*x + p2 - angleDesired; 
end



