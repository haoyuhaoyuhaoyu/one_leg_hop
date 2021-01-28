clear variables;
close all;
syms pcx pcy sita real;
syms dpcx dpcy dsita real;
syms ddpcx ddpcy ddsita real;
syms fx fy pex pey real;
syms g m I positive;

%%%% dynamics
% dL/dt = r cross ma = tau
eq1=cross([pcx;pcy;0],m*[ddpcx;ddpcy;0])+ ...
    I*[0;0;ddsita]==cross([pex;pey;0],[fx;fy;0])+ ...
    cross([pcx;pcy;0],[0;-m*g;0]);
% ma = F
eq2=m*[ddpcx;ddpcy]==[fx;fy]+[0;-m*g];

%%%% evaluating the results of : MM*ddq=ff
ddq=[ddpcx;ddpcy;ddsita];
eqs=[eq1;eq2];
[MM,ff]=equationsToMatrix(eqs,ddq);
dyn=simplify(MM\ff);

%%%% generate an optimized matlab function for dynamics
matlabFunction(dyn(1),dyn(2),dyn(3), ...
    'file','autoGen_hoppingDynamics',...
    'vars',{pcx,pcy,sita,dpcx,dpcy,dsita,pex,pey,fx,fy,m,g,I}, ...
    'outputs',{'ddpcx','ddpcy','ddsita'});