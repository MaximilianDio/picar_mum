clear
close all
clc

%% Analytische Loesung fuer eine stationaere Kreisfahrt

syms k_sv k_sh v delta_v

% Radius des Kreises
R = 1;
m = 2.3228;
lv = 0.1393;
lh = 0.1207;

% 'Dampingmatrix'
A = [(k_sv + k_sh), (m*v + (k_sv*lv - k_sh*lh)/v);...
     (k_sv*lv - k_sh*lh), (k_sv*lv^2 + k_sh*lh^2)/v];
 
% Right hand side
b = [k_sv*delta_v;...
     k_sv*lv*delta_v];
 
% Predicted yaw rate dpsi
dpsi = simplify([0 1] * (A\b));

matlabFunction(dpsi,'File','dpsi_Function','Vars',[k_sv k_sh v delta_v]);