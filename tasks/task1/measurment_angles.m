%%
clear
close all
clc
%%
anglesIn = 0:5:30;
anglesOut = [0;5;9;13;18;23;26];

plot(anglesIn,anglesOut);
hold on;
plot(anglesIn,G_angles(anglesIn));
legend("measured Data","linear fitting curve");

xlabel("angles Input in Deg");
ylabel("angles Output in Deg")


% angeDesiredIn = fsolve(@G_angles,15);

%% linear interpolation
function y = G_angles(x)
    angleDesired = 0;%14.95; %in deg
    
    % Coefficients:
      p1 = 0.87857;
      p2 = 0.25;

    y = p1*x + p2 - angleDesired; 
end



