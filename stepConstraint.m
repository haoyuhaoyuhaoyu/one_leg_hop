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
% x = [6,1] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
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
c1 = -dpcx0 + 0.1;
c2 = dpcy0 + 0.02;
c = [c1; c2];

% ceq
ceq1 = pcxF - pcx0 - step_length;
ceq2 = pcyF - pcy0 - step_height;
ceq3 = sitaF - sita0;
c_v = [dpcx0-dpcxF; dpcy0-dpcyF;dsita0-dsitaF];
ceq  = [ceq1; ceq2; ceq3; c_v];

end