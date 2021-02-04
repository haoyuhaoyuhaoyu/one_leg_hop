% derive dynamics through lagrange
clear variables;
close all;
syms pcx pcy sita real;
syms dpcx dpcy dsita real;
syms ddpcx ddpcy ddsita real;
syms fx fy pex pey real;
syms g m I positive;

% q = [pcx; pcy; sita]
% tau = cross([pex-pcx;pey-pcy;0], [fx;fy;0])

% compute T
Jv = [1,0,0; 0,1,0; 0,0,0];
Jw = [0,0,1];
q = [pcx; pcy; sita];
dq = [dpcx; dpcy; dsita];
ddq = [ddpcx; ddpcy; ddsita];
T = 0.5*m*dq'*Jv'*Jv*dq + 0.5*dq'*Jw'*I*Jw*dq;

% compute U
U = m*g*pcy;

% compute Lagrange function and generalized force
L = simplify(T - U);
tau = [fx; fy; [0,0,1]*cross([pex-pcx;pey-pcy;0], [fx;fy;0])];

% Lagrange eqn
temp = jacobian(L, dq)';
element1 = jacobian(temp, dq)*ddq;
element2 = jacobian(L, q)';
eqn = simplify(element1 - element2) == tau;


[MM,ff]=equationsToMatrix(eqn,ddq);
dyn=simplify(MM\ff);

% generate an optimized matlab function for dynamics
matlabFunction(dyn(1),dyn(2),dyn(3), ...
    'file','autoGen_hoppingDyn_lagrange',...
    'vars',{pcx,pcy,sita,dpcx,dpcy,dsita,pex,pey,fx,fy,m,g,I}, ...
    'outputs',{'ddpcx','ddpcy','ddsita'});
