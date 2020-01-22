% Dynamics:
%   A simple model of a car, where the state is its position and
%   orientation, and the control is the input velocity and the steering
%   angle
%
% Objective:
%   Find the best path for an overtaking maneuver with the minimum control
%   effort

close all
clear
clc

xBnd = [-5, 5];
yBnd = [-10, 10];

startPoint = [0; 0; 0];   %Start here
finishPoint = [2; 0; 0];   %Finish here


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Set up function handles                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( BycicleModel(t,x,u) );
problem.func.pathObj = @(t,x,u)(sum([100 * u(1,:); u(2,:)].^2,1));
problem.func.pathCst = @(t,x,u)( pathConstraint(t,x,u) );
problem.func.bndCst = @(t0,x0,tF,xF,u0,uF)( boundaryConstraint(t0, x0, tF, xF,u0,uF) );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Set up bounds on state and control                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 1.5;
problem.bounds.finalTime.upp = 4;

problem.bounds.state.low = [xBnd(1); yBnd(1); -2*pi];
problem.bounds.state.upp = [xBnd(2); yBnd(2);  2*pi];

problem.bounds.initialState.low = startPoint;
problem.bounds.initialState.upp = startPoint;

problem.bounds.finalState.low = finishPoint;
problem.bounds.finalState.upp = finishPoint;

problem.bounds.control.low = [0.8; - deg2rad(25)];
problem.bounds.control.upp = [1.5; deg2rad(25)];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Initialize trajectory with guess                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Car travels at a speed of one, and drives in a straight line from start
% to finish point.

problem.guess.time = [0, 5];   % time = distance/speed
problem.guess.state = [startPoint, finishPoint];
problem.guess.control = zeros(2);  


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Options for Transcription                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'display','iter',...
    'MaxFunEval',1e5,...
    'tolFun',1e-6);

% problem.options.method = 'hermiteSimpson';
% problem.options.hermiteSimpson.nSegment = 25;

% problem.options.method = 'gpops';

problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 20;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

velocities = BycicleModel(soln.grid.time, soln.grid.state, soln.grid.control);

%%
L = 0.1207 + 0.1393;
t_sim = linspace(soln.grid.time(1), soln.grid.time(end), 500);
state_sim = soln.interp.state(t_sim);
control_sim = soln.interp.control(t_sim);

A = struct('L', L, 't', t_sim, 'theta', state_sim(3,:), 'u', control_sim,...
            'x', state_sim(1,:), 'y', state_sim(2,:));
        
save('Animation.mat', 'A')

