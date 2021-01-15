function [c, ceq] = pathConstraint(x, u)
% [c, ceq, cGrad, ceqGrad] = pathConstraint(x)
%
% This function implements a simple path constraint to keep the knee joint
% of the robot from hyer-extending.
%
% x = [6,n] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
% uEx = [4,n] = [pex;pey;fx;fy] = supporting point and forces

miu = 0.8; % for friction
L_max = 0.6;
L_min = 0.4;
[~, grid_num] = size(x);
phase_separate = fix(grid_num/2); % stance and swing 

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

LL = zeros(phase_separate,1); % Leg Length
leg_sita = zeros(phase_separate,1); % Leg sita
% for c constr
c_MaxLL = zeros(grid_num,1);
c_MinLL = zeros(grid_num,1);
c_Cone = zeros(phase_separate,1);
c_SF = zeros(phase_separate,1);
c_SwingSPy = zeros(phase_separate,1);

% for ceq constr
ceq2 = zeros(phase_separate,1);
ceq5 = zeros(phase_separate,1);
ceq6 = zeros(phase_separate,1);
ceq3 = zeros(phase_separate,1);
ceq4 = zeros(phase_separate,1);

% ce constr
% all phase
% leg length constr
for i=1:1:grid_num
    %for stance phase constr
    pc = [pcx(i), pcy(i)];
    pe = [pex(i), pey(i)];
    LL_vec = pc - pe; % peF = pcF - LL_vecF 
    LL(i) = norm(LL_vec);
    c_MaxLL(i) = LL(i) - L_max;
    c_MinLL(i) = -LL(i) + L_min;
end

% stance phase
for i=1:1:phase_separate
    %for stance phase constr
    pc = [pcx(i), pcy(i)];
    pe = [pex(i), pey(i)];
    leg_sita(i) = atan((pc(1)-pe(1))/(pc(2)-pe(2)));
    % friction cone
    c_Cone(i) = fx(i)-fy(i)*miu*cos(leg_sita(i));
    % support force always > 0
    c_SF(i) = -fy(i);
end
% Support Point ahead of COM
c_SP = -pex(1) + pcx(1) +0.1;
c_stance = [c_MaxLL; c_MinLL; c_Cone; c_SF; c_SP];
c_stance = reshape(c_stance, numel(c_stance), 1);

% swing phase
for i = phase_separate+1:1:grid_num
    % for swing phase constr
    c_SwingSPy = -pey(i); 
end
c_swing = c_SwingSPy;
c_swing = reshape(c_swing, numel(c_swing), 1);
% all c constr
c = [c_stance; c_swing];

% ceq constr
% stance point constr, 
LL_vec0 = [pcx(1), pcy(1)] - [pex(1), pey(1)];
peF = [pex(grid_num), pey(grid_num)];
pcF = [pcx(grid_num), pcy(grid_num)];
ceq_LL = norm(LL_vec0-(pcF-peF));
% stance phase
for i=1:1:phase_separate
    %for stance phase constr
    ceq2(i) = pey(i);
    ceq5(i) = pex(i) - pex(1);
    ceq6(i) = pex(i);
end
% swing phase
for i=phase_separate+1:1:grid_num
    %for stance phase constr
    ceq3(i) = fx(i);
    ceq4(i) = fy(i);
end
ceq2 = reshape(ceq2, numel(ceq2), 1);
ceq = [ceq_LL; ceq2; ceq3; ceq4; ceq5];

end