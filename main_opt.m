%setup path
clear; close all; clc;
restoredefaultpath; matlabrc;

Optim_path = '../../OptimTraj';
addpath(genpath(Optim_path));

%init
p.m = 3;  % (kg) robot mass
p.g = 9.81;  % (m/s^2) gravity
p.I = 0.5;   % (kg*m^2)inertia

p.stepLength = 1;
p.stepTime = 0.7;
p.stepHeight = 0.25;

p.user_grid = 60;
%%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.func.dynamics = @(t,x,u)( hoppingDynamics(x,u,p) );
problem.func.pathObj = @(t,x,u)( obj_torque(x, u) );  % minimize u
problem.func.pathCst = @(t,x,u)( pathConstraint(x, u, p) );
problem.func.bndCst = @(t0,x0,tF,xF)( stepConstraint(x0,xF,p) );

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = p.stepTime;
problem.bounds.finalTime.upp = p.stepTime;

problem.bounds.initialState.low = [-0.5;0.5;-pi/4;-100;-100;-100];
problem.bounds.initialState.upp = [0;0.8;pi/4;100;100;100];
problem.bounds.finalState.low = [0.5;0.5;-pi/4;-100;-100;-100];
problem.bounds.finalState.upp = [0.7;1.2;pi/4;100;100;100]; 

problem.bounds.state.low = [-5;0.5;-pi/4;-inf;-inf;-inf];
problem.bounds.state.upp = [5;1.2;pi/4;inf;inf;inf];

problem.bounds.control.low = [-5;0;-inf;0];
problem.bounds.control.upp = [5;0.5;inf;p.m*p.g*10];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% For now, just assume a linear trajectory between boundary values

problem.guess.time = [0, p.stepTime];
problem.guess.state = [[0;0.1;-0.15;2.3;-2.17;-2.16], [1;0.6;-0.15;2.3;-2.17;-2.16]];
problem.guess.control = [[0;0;-13;73],[0.7;0.15;0;0]];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter-detailed',...
    'TolFun',1e-3,...
    'MaxFunEvals',3e5);

problem.options.method = 'trapezoid';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem, p.user_grid);
save('soln','soln');

%%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %                                     
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
step_Num = 2;
draw_MultiStep(soln, p, step_Num);
%%
[check_pathC, check_pathCeq, check_stepC, check_stepCeq, ...
    check_Dyn] = check_Constr(soln, p);

%%
% dsita = soln.grid.state(6,:);
% plot(dsita);
% figure()
% sita = soln.grid.state(3,:);
% plot(sita);