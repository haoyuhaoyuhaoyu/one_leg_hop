%setup path
clear; close all; clc;
restoredefaultpath; matlabrc;

Optim_path = '../../OptimTraj';
addpath(genpath(Optim_path));

%init
p.m = 3;  % (kg) robot mass
p.g = 9.81;  % (m/s^2) gravity
p.I = 0.5;   % (kg*m^2)inertia

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.func.dynamics = @(t,x,u)( hoppingDynamics(x,u,p) );
problem.func.pathObj = @(t,x,u)( u(1,:).^2 + u(2,:).^2 );  % minimize u
problem.func.pathCst=@(phaseT,SP,x,u)(collectPathConstrain(phaseT,SP,x,u));

