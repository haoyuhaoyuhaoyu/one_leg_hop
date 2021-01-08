function [c, ceq] = pathConstraint(x, u)
% [c, ceq, cGrad, ceqGrad] = pathConstraint(x)
%
% This function implements a simple path constraint to keep the knee joint
% of the robot from hyer-extending.
%
% x = [6,n] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
% uEx = [4,n] = [pex;pey;fx;fy] = supporting point and forces
[~, grid_num] = size(x);

phase_separate = fix(grid_num/2);

LL = zeros(phase_separate,1);
c1 = zeros(phase_separate,1);
c2 = zeros(phase_separate,1);
c3 = zeros(phase_separate,1);
c4 = zeros(phase_separate,1);
ceq2 = zeros(phase_separate,1);
L_max = 0.7;
L_min = 0.4;

pcx   = x(1,:);
pcy   = x(2,:);
sita  = x(3,:);
dpcx  = x(4,:);
dpcy  = x(5,:);
dsita = x(6,:);
pex   = u(1,:);
pey   = u(2,:);
% ce constr
% stance phase
for i=1:1:phase_separate
    %for stance phase constr
    pc = [pcx(i), pcy(i)];
    pe = [pex(i), pey(i)];
    LL_vec = pc - pe; % peF = pcF - LL_vecF 
    LL(i) = norm(LL_vec);
    c1(i) = LL(i) - L_max;
    c2(i) = -LL(i) + L_min;
end

% swing phase
for i = phase_separate+1:1:grid_num
    % for swing phase constr
    if i > fix(grid_num/2) + phase_separate
       c3(i) = 0.15 - pey(i);
       c4(i) = 0.35 - pex(i);
    end
end
c_stance = [c1;c2];
c_stance = reshape(c_stance, numel(c_stance), 1);
c_swing = [c3; c4];
c_swing = reshape(c_swing, numel(c_swing), 1);
c = [c_stance; c_swing];
% ceq constr
% stance point constr
LL_vec0 = [pcx(1), pcy(1)] - [pex(1), pey(1)];
peF = [pex(grid_num), pey(grid_num)];
pcF = [pcx(grid_num), pcy(grid_num)];
ceq1 = norm(LL_vec0-(pcF-peF));
% stance phase
for i=1:1:phase_separate
    %for stance phase constr
    ceq2(i) = pey(i);
end
ceq2 = reshape(ceq2, numel(ceq2), 1);
ceq = [ceq1; ceq2];

end