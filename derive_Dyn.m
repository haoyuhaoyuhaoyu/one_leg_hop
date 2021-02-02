clear variables;
close all;
syms pcx pcy sita real;
syms dpcx dpcy dsita real;
syms ddpcx ddpcy ddsita real;
syms fx fy pex pey real;
syms g m I positive;

% F = Ma                              Newton equ
eq1 = [fx;fy;0] + [0;-m*g;0] == m*[ddpcx; ddpcy; 0];
% M = I*alpha + w¡ÁIw                 Euler equ
eq2 = cross([pex-pcx;pey-pcy;0],[fx; fy; 0]) == ...
    I*[0;0;ddsita]; 

% evaluating the results of : MM*ddq=ff
ddq=[ddpcx;ddpcy;ddsita];
eqs=[eq1;eq2];
[MM,ff]=equationsToMatrix(eqs,ddq);
dyn=simplify(MM\ff);

% generate an optimized matlab function for dynamics
matlabFunction(dyn(1),dyn(2),dyn(3), ...
    'file','autoGen_hoppingDyn',...
    'vars',{pcx,pcy,sita,dpcx,dpcy,dsita,pex,pey,fx,fy,m,g,I}, ...
    'outputs',{'ddpcx','ddpcy','ddsita'});