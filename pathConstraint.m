function [c, ceq] = pathConstraint(x, u)
% [c, ceq, cGrad, ceqGrad] = pathConstraint(x)
%
% This function implements a simple path constraint to keep the knee joint
% of the robot from hyer-extending.
%
% x = [6,n] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
% uEx = [4,n] = [pex;pey;fx;fy] = supporting point and forces

miu = 0.8; % for friction
L_max = 0.65;
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
% c_stance = [c_MaxLL; c_MinLL; c_Cone; c_SF; c_SP];
c_MaxLL = zeros(grid_num,1);
c_MinLL = zeros(grid_num,1);
c_Cone = zeros(phase_separate,1);
c_SF = zeros(phase_separate,1);
c_SwingSPy = zeros(phase_separate,1);
c_SwingSita = zeros(phase_separate,1);

% for ceq constr
ceq_SPy = zeros(phase_separate,1);
ceq_SPx = zeros(phase_separate,1);
ceq_SwFx = zeros(phase_separate,1);
ceq_SwFy = zeros(phase_separate,1);

% ce constr
% all phase
% leg length constr
for i=1:1:grid_num
    %for all phase constr
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
    %c_Cone(i) = fx(i) - fy(i)*miu*tan(leg_sita(i));
    c_Cone(i) = fx(i)^2 - (fy(i)*miu)^2;
    % support force always > 0
    c_SF(i) = -fy(i);
end
% Support Point ahead of COM
c_SP = -pex(1) + pcx(1) + 0.2;
c_stance = [c_MaxLL; c_MinLL; c_Cone; c_SF; c_SP];
c_stance = reshape(c_stance, numel(c_stance), 1);

% swing phase
for i = phase_separate+1:1:grid_num
    % for swing phase constr
    % when swing, SP y must > 0
    c_SwingSPy(i-phase_separate) = -pey(i); 
%     
    c_SwingSita(i-phase_separate) = sita(phase_separate);
end
% c_swing = [c_SwingSPy; c_SwingSita];
c_swing = [c_SwingSPy; c_SwingSita];
% all c constr
c = [c_stance; c_swing];

% ceq constr
% for init SPx£¬ support point is [0,0]
ceq_SPx0 = pex(1); 
% stance point constr, x0,xF the same Leg Length
LL_vec0 = [pcx(1), pcy(1)] - [pex(1), pey(1)];
peF = [pex(grid_num), pey(grid_num)];
pcF = [pcx(grid_num), pcy(grid_num)];
ceq_LL = norm(LL_vec0-(pcF-peF));
% stance phase
for i=1:1:phase_separate
    %for stance phase constr
    ceq_SPy(i) = pey(i); % y is on the ground
    ceq_SPx(i) = pex(i) - pex(1); % x cannot move
end
% swing phase
for i=phase_separate+1:1:grid_num
    %for stance phase constr
    ceq_SwFx(i-phase_separate) = fx(i);
    ceq_SwFy(i-phase_separate) = fy(i);
end
ceq = [ceq_SPy; ceq_SPx; ceq_SwFx; ceq_SwFy; ceq_SPx0; ceq_LL];
end