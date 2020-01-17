function [c, ceq, cGrad, ceqGrad] = pathConstraint(t, x, u)

% smooth control input
c = [];

for i = 1:length(u)-1
    c = [c; abs(u(:,i+1) - u(:,i)) - [0.4;pi/5]];
end

% Box on the road

% Length
L = 0.15;

% width
w = 0.15;

% Position of the box
p = [0.75; 0];
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