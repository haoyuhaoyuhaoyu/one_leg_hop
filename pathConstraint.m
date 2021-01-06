function [c, ceq] = pathConstraint(x, p)
% [c, ceq, cGrad, ceqGrad] = pathConstraint(x)
%
% This function implements a simple path constraint to keep the knee joint
% of the robot from hyer-extending.
%
% x = [6,n] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
% uEx = [4,n] = [pex;pey;fx;fy] = supporting point and forces
step_length = p.stepLength;
[~, time_grid] = size(x);
pcx   = x(1,:);
pcy   = x(2,:);
sita  = x(3,:);
dpcx  = x(4,:);
dpcy  = x(5,:);
dsita = x(6,:);

stand_time_index = fix(time_grid/2);

c1 = -pcy;
c2 = 0.15 - pcy(stand_time_index);
c3 = -dpcy(stand_time_index);
c = [c1; c2; c3];
ceq = [];


end