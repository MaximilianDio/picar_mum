%% measurments
%% constants

g = 9.81; %m/s^2
%% mass

massCar = [1902.84; 1902.33 ; 1902.34]*10^-3; %kg Gesamtmasse Auto ohne Akku
    
massAkku = [ 420.35; 420.33 ; 420.34]*10^-3; %kg Gesmatmasse Akku

mges = mean(massCar)+mean(massAkku); %kg Gesmatmasse
%% CoM

%% x-direction
 
Fx = [ 8.2 ; 8.2; 8.2]; %N 


L = 0.295; %m %Gesmatl?nge 
Lx = 0.0755; %m L?nge von Vorderachse zu Kraftangriffspunkt 
LF = L+Lx; %m Angriffspunkt Kraft F

Lh = mean(Fx)*LF/(mges*g);
Lv = L-Lh;

%% y-direction

% Kraft ohne Akku
Fy = [9.1; 9.5; 9.4; 9.4; 9.4]; %N 

% Breite Felge-Felge
b = 0.16; % m 

% Abstand Angriffspunkt von rechter Felge
bF = 0.065+b/2; %m 

% Abstand Schwerpunkt Akku von rechter Felge
bAkku = 0.033+b/2; %m 

% Abstand Schwerpunkt Auto ohne Akku zu rechter Felge
brCar = mean(Fy)*bF/((mean(massCar))*g); %m

% Abstand Schwerpunkt Auto mit Akku zu rechter Felge
br = (mean(massAkku)*bAkku+mean(massCar)*brCar)/mges; %m

%% z-direction

% Kraft
Fz = 3.7; %N

% Höhe Kraftangriffspunkt 
hF = 0.298-0.032; %m

% Schwerpunkt gesamtmasse Auto mit Akku
h = hF*Fz/(mges*g); %m 

%% Massentr?gheit

% Aufbau:

% Abstand con der Drehachse zum unteren Teil der 'Schwingbuehne'
LAufbau = 0.518; %m

% Masse der Schwingbuehne
masseAufbau = (597.4+1206.95)*(10^-3); %kg

% Kraft im statischen Gleichgewicht
FAufbau = 5.65; %N
% Hebelarm
LFAufbau = 0.255; %m

% Schwerpunkt der Schwingbuehne dargestellt im Schwingbuehnen KOS
zAufbau = LFAufbau *FAufbau/(masseAufbau*g); %m

% Abstand des Schwerpunktes der Schwingbuehne zur Drehachse
LAufbauSchwerpunkt = LAufbau-zAufbau; %m 

% Periodendauer der Schwingbuehne
T_Aufbau = 1.444; %s

%Tr?gheitsmoment (bzgl. Drehachse) berechnet sich aus der Eigenfrequenz der Schwingbuehne(Pendelschwingung)
I_Aufbau = (masseAufbau*g*LAufbauSchwerpunkt)/((2*pi/T_Aufbau)^2);

% Messung fuer Schingbuehne und Auto
% Gesamtschwerpunkt im Schwingbuehnen KOS
z_ges = (masseAufbau * zAufbau + mges * 0.135)/(masseAufbau + mges); % 13.5cm wurden grob gemessen

% Abstand Gesamtschwerpunkt zur Drehachse
L_CoM_ges = LAufbau - z_ges;

% Periodendauer gesamtsystem
T_ges = 1.372;

% Abstand von der Drehachse zum Schwerpunkt des Autos
L_CoM_Auto = LAufbau - 0.135;

I_Auto = ((masseAufbau + mges)*g*L_CoM_ges)/((2*pi/T_ges)^2) - I_Aufbau - mges*L_CoM_Auto^2; %kg m^2


