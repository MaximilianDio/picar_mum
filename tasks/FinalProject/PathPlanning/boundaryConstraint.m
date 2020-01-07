function [c, ceq, cGrad, ceqGrad] = boundaryConstraint(t0,x0,tF,xF, u0, uF)

v_init = 0.3;
v_end = 0.5;
ceq = [[1,0,0] * BycicleModel(tF, xF, uF) - v_end;...
        [1,0,0] * BycicleModel(t0, x0, u0) - v_init];

% ceq = [ceq; xF(1) - x0(1) - 0.3];

c = [];
cGrad = [];
ceqGrad = [];

end