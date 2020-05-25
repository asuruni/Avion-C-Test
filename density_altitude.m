function [density] = density_altitude(h)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
p_o = 101.325; %kPa
T_o = 288.15; %K
g=9.80665; %m/s^2
L=0.0065; %K/m
R=8.31447; %J/mol*K
M=0.0289644; %kg/mol

p=p_o*(1-(L*h/T_o))^(g*M/(R*L));
T=(T_o-L*h);
density=1000*(M*p)/(R*T);

end

