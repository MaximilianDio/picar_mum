clear
close all
clc


%% Skript zum Ermitteln der Seitenkraftbeiwerte k_sh und k_sv


filenames = ["Vel03Stear165.txt";
             "Vel06Stear1775.txt";
             "Vel09Stear185.txt"];

angle_data=zeros(3,1);
velocity_data = zeros(3,1);


%compute accels


Mat1=GetYawRate(filenames(1));%import data
time1 = Mat1(:,1);
dpsi1 = Mat1(:,2);
time1=time1-time1(1); %correct time

Mat2=GetYawRate(filenames(2));%import data
time2 = Mat2(:,1);
dpsi2 = Mat2(:,2);
time2=time2-time2(1);

Mat3=GetYawRate(filenames(3));%import data
time3 = Mat3(:,1);
dpsi3 = Mat3(:,2);
time3=time3-time3(1);

% cut off zero entries
% First Experiment (824:4163)
dpsi1_const = mean(dpsi1(824:4163)) * pi/180;
dpsi2_const = mean(dpsi2(330:1360)) * pi/180;
dpsi3_const = mean(dpsi3(310:900)) * pi/180;

dpsi_meas = [dpsi1_const; dpsi2_const; dpsi3_const];


%compute angles
angles_commanded=[16.5;17.75;18.5];
for n=1:length(angles_commanded)
angle_data(n)=G_angles(angles_commanded(n));
end
velocity_commanded = [0.3 0.6 0.9];
for n=1:length(velocity_commanded)
velocity_data(n)=G_speed(velocity_commanded(n));
end

% Define estimation problem for fmincon

% Initial Guess
x0 = 80 * 1e3 .* ones(2,1);
A = [-ones(2); ones(2)];
b = [100 100 1e5 1e5].';

x = fmincon(@(X)EstimationError(X, velocity_data, angle_data, dpsi_meas), x0, A, b);

 
EstimationError(x, velocity_data, angle_data, dpsi_meas)
% Function handle estimation errors
function J = EstimationError(x, Vel, Angle, yaw_meas)
    J = 0;
    for i = 1:3
        J = J + norm(yaw_meas(i) - dpsi_Function(x(1), x(2), Vel(i), Angle(i)))^2;
    end
end

function y = G_angles(x)
    angleDesired = 0;%14.95; %in deg
    
    % Coefficients:
      p1 = 0.87857;
      p2 = 0.25;

    y = p1*x + p2 - angleDesired; 
end

function y = G_speed(x)

    % Coefficients:
      p1 = -0.76149;
      p2 = 2.6308;
      p3 = -0.3301;


    y = p1*x.^2 + p2*x + p3; 

end