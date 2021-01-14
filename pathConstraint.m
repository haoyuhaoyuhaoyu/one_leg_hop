function [c, ceq] = pathConstraint(x, u)
% [c, ceq, cGrad, ceqGrad] = pathConstraint(x)
%
% This function implements a simple path constraint to keep the knee joint
% of the robot from hyer-extending.
%
% x = [6,n] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
% uEx = [4,n] = [pex;pey;fx;fy] = supporting point and forces

miu = 0.8;
[~, grid_num] = size(x);

phase_separate = fix(grid_num/2);

LL = zeros(phase_separate,1);
leg_sita = zeros(phase_separate,1);
c1 = zeros(grid_num,1);
c2 = zeros(grid_num,1);
c3 = zeros(phase_separate,1);
c4 = zeros(phase_separate,1);
c_f = zeros(phase_separate,1);
c_fy = zeros(phase_separate,1);
c_swing_y = zeros(phase_separate,1);
ceq2 = zeros(phase_separate,1);
ceq5 = zeros(phase_separate,1);
ceq3 = zeros(phase_separate,1);
ceq4 = zeros(phase_separate,1);
L_max = 0.6;
L_min = 0.4;

pcx   = x(1,:);
pcy   = x(2,:);
sita  = x(3,:);
dpcx  = x(4,:);
dpcy  = x(5,:);
dsita = x(6,:);
pex   = u(1,:);
pey   = u(2,:);
fx    = u(3,:);
fy    = u(4,:);
% ce constr
% all phase
for i=1:1:grid_num
    %for stance phase constr
    pc = [pcx(i), pcy(i)];
    pe = [pex(i), pey(i)];
    LL_vec = pc - pe; % peF = pcF - LL_vecF 
    LL(i) = norm(LL_vec);
    c1(i) = LL(i) - L_max;
    c2(i) = -LL(i) + L_min;
end
% stance phase
for i=1:1:phase_separate
    %for stance phase constr
%     pc = [pcx(i), pcy(i)];
%     pe = [pex(i), pey(i)];
%     LL_vec = pc - pe;  
%     LL(i) = norm(LL_vec);
%     c1(i) = LL(i) - L_max;
%     c2(i) = -LL(i) + L_min;
    pc = [pcx(i), pcy(i)];
    pe = [pex(i), pey(i)];
    leg_sita(i) = atan((pc(1)-pe(1))/(pc(2)-pe(2)));
    c_f(i) = fx(i)-fy(i)*miu*cos(leg_sita(i));
    c_fy(i) = -fy(i);
end

% swing phase
for i = phase_separate+1:1:grid_num
    % for swing phase constr
    c_swing_y = -pey(i); 
%     if i > fix(grid_num/2) + phase_separate
%        c3(i) = 0.15 - pey(i);
%        c4(i) = 0.35 - pex(i);
%     end
end

%c_0 = -pex(1) + 0.1;
c_stance = [c1;c2;c_f;c_fy];
c_stance = reshape(c_stance, numel(c_stance), 1);
c_swing = [c3; c4; c_swing_y];
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
    ceq5(i) = pex(i) - pex(1);
end
% swing phase
for i=phase_separate+1:1:grid_num
    %for stance phase constr
    ceq3(i) = fx(i);
    ceq4(i) = fy(i);
end
ceq2 = reshape(ceq2, numel(ceq2), 1);
ceq = [ceq1; ceq2; ceq3; ceq4; ceq5];

end