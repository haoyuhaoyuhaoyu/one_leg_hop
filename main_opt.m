%setup path
clear; close all; clc;
restoredefaultpath; matlabrc;

Optim_path = '../../OptimTraj';
addpath(genpath(Optim_path));

%init
p.m = 3;  % (kg) robot mass
p.g = 9.81;  % (m/s^2) gravity
p.I = 0.5;   % (kg*m^2)inertia

p.stepLength = 0.7;
p.stepTime = 0.5;
p.stepHeight = 0.15;

p.user_grid = 60;
%%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.func.dynamics = @(t,x,u)( hoppingDynamics(x,u,p) );
problem.func.pathObj = @(t,x,u)( obj_torque(x, u) );  % minimize u
problem.func.pathCst = @(t,x,u)( pathConstraint(x, u) );
problem.func.bndCst = @(t0,x0,tF,xF)( stepConstraint(x0,xF,p) );

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = p.stepTime;
problem.bounds.finalTime.upp = p.stepTime;

problem.bounds.initialState.low = [-0.2;0.5;-pi/6;-100;-100;-100];
problem.bounds.initialState.upp = [0;0.8;pi/6;100;100;100];
problem.bounds.finalState.low = [0.5;0.5;-pi/6;-100;-100;-100];
problem.bounds.finalState.upp = [0.7;1.2;pi/6;100;100;100]; 

problem.bounds.state.low = [-5;0.5;-pi/6;-inf;-inf;-inf];
problem.bounds.state.upp = [5;1.2;pi/6;inf;inf;inf];

problem.bounds.control.low = [-5;0;-inf;0];
problem.bounds.control.upp = [5;1.2;inf;p.m*p.g*10];


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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %                                     
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%
state = soln.grid.state;
control = soln.grid.control;

pcx = state(1,:);
pcy = state(2,:);
sita = state(3,:);
dsita = state(6,:);
pex = control(1,:);
pey = control(2,:);

phase_separate = fix(p.user_grid/2);
pC = [p.stepLength/2, p.stepHeight*1.3];
p0 = [pex(phase_separate), pey(phase_separate)];
pF = [pex(p.user_grid), pey(p.user_grid)];

[x,y] = interp(p0,pF,pC,phase_separate);
pex(phase_separate+1:end) = x;
pey(phase_separate+1:end) = y;
for i=1:1:60
    hold off
    plot([pcx(i),pex(i)],[pcy(i),pey(i)],'LineWidth',2);
    hold on
    draw_stair(p.stepLength, p.stepHeight);
    hold on
    draw_robot(sita(i), pcx(i),pcy(i));
    hold on
    plot(pcx(i),pcy(i),'o',...
        'MarkerSize',5);
    axis equal
    xlim([-2,2]);
    ylim([-0.5,2.5]);  
    hold on
    drawnow;
    pause(0.08);
end
%%
[check_pathC, check_pathCeq, check_stepC, check_stepCeq, ...
    check_Dyn] = check_Constr(soln, p);