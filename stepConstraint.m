function [c, ceq] = stepConstraint(x0,xF,p)
% [c, ceq] = stepConstraint(x0,xF,p)
%
% INPUTS:
%   t0 = time at the start of the trajectory
%   x0 = state at the start of the trajectory
%   tF = time at the end of the trajectory
%   xF = state at the end of the trajectory
%
% OUTPUTS:
%   c = inequality constraint
%   cGrad = gradient of inequality constraints       
%   x = [6,1] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
step_length = p.stepLength;
step_height = p.stepHeight;
pcx0   = x0(1);
pcy0   = x0(2);
sita0  = x0(3);
dpcx0  = x0(4);
dpcy0  = x0(5);
dsita0 = x0(6);

pcxF   = xF(1);
pcyF   = xF(2);
sitaF  = xF(3);
dpcxF  = xF(4);
dpcyF  = xF(5);
dsitaF = xF(6);

% ce
% for the com vx
c_vx0 = -dpcx0;
% for the com vy
c_vy0 = dpcy0;
c = [c_vx0; c_vy0];

% ceq
%  for step length
ceq1 = pcxF - pcx0 - step_length;
%  for step height
ceq2 = pcyF - pcy0 - step_height;
%  for body sita
ceq3 = sitaF - sita0;
%  suppose the init and end state have the same velocity
ceq_v = [dpcx0-dpcxF; dpcy0-dpcyF; dsita0-dsitaF];
%ceq  = [ceq1; ceq2; ceq3; ceq_v];
ceq  = [ceq1; ceq2; ceq3; ceq_v];
end