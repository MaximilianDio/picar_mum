function [c, ceq, cGrad, ceqGrad] = pathConstraint(t, x, u)

% smooth control input
c = [];

for i = 1:length(u)-1
    c = [c; (u(:,i+1) - u(:,i)) - [0.05;pi/40]];
end

% Box on the road

% Length
L = 0.1;

% width
w = 0.1;

% Position of the box
p = [1; 0];
% Circle around box with radius
r = sqrt(L^2 + w^2);
% 
for i = 1: length(x)
    c = [c; -norm(x(1:2,i) - p) + r];
end

ceq = [];
cGrad = [];
ceqGrad = [];
end