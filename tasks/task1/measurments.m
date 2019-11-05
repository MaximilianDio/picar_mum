%% measurments
close all
clear
clc

%% constants

g = 9.81; %m/s^2
%% mass

massCar = [1902.84; 1902.33 ; 1902.34]*10^-3; %kg Gesamtmasse Auto ohne Akku
    
massAkku = [ 420.35; 420.33 ; 420.34]*10^-3; %kg Gesmatmasse Akku


mges = mean(massCar)+mean(massAkku); %kg Gesmatmasse
%% CM

%% x-direction
 
Fx = [ 8.2 ; 8.2; 8.2]; %N 


L = 0.295; %m %Gesmatl�nge 
Lx = 0.0755; %m L�nge von Vorderachse zu Kraftangriffspunkt 
LF = L+Lx; %m Angriffspunkt Kraft F

Lh = mean(Fx)*LF/(mges*g);
Lv = L-Lh;

%% y-direction

Fy = [9.1; 9.5; 9.4; 9.4; 9.4]; %N %kraft ohne Batterie

b = 0.16; % m Breite Felge-Felge

bF = 0.065+b/2; %m Angriffspunkt von rechter Felge

bAkku = 0.033+b/2; %m Schwerpunkt Akku von rechter Felge

brCar = mean(Fy)*bF/((mean(massCar))*g); %m Schwerpunkt Auto ohne Akku von rechter Felge

br = (mean(massAkku)*bAkku+mean(massCar)*brCar)/mges; %m Schwerpunkt Auto mit Akku von rechter Felge

%% z-direction

Fz = 3.7; %N
hF = 0.298-0.032; %m

h = hF*Fz/(mges*g); %m Schwerpunkt gesamtmasse Auto mit Akku

%% Massentr�gheit

% Aufbau:

LAufbau = 0.518; %m

masseAufbau = (597.4+1206.95)*(10^-3); %kg

FAufbau = 5.65; %N
LFAufbau = 0.255; %m

zAufbau = LFAufbau *FAufbau/(masseAufbau*g); %m %Schwerpunkt von Drehpunkt

% Schwingung
%Abstand Drehpunkt zu Schwerpunkt
LAufbauSchwerpunkt = LAufbau-zAufbau; %m 