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
finishPoint = [1; 0; 0];   %Finish here


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Set up function handles                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( BycicleModel(t,x,u) );
problem.func.pathObj = @(t,x,u)(sum(u.^2,1));
problem.func.pathCst = @(t,x,u)( pathConstraint(x) );
problem.func.bndCst = @(t0,x0,tF,xF)( stepConstraint(x0,xF,param) );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Set up bounds on state and control                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 2;
problem.bounds.finalTime.upp = 100;

problem.bounds.state.low = [xBnd(1); yBnd(1); -2*pi];
problem.bounds.state.upp = [xBnd(2); yBnd(2);  2*pi];

problem.bounds.initialState.low = startPoint;
problem.bounds.initialState.upp = startPoint;

problem.bounds.finalState.low = finishPoint;
problem.bounds.finalState.upp = finishPoint;

problem.bounds.control.low = [0; - pi/2];
problem.bounds.control.upp = [10; pi/2];


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
problem.options.trapezoid.nGrid = 15;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);


