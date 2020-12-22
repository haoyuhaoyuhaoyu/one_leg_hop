function dz = hoppingDynamics(z,uEx,p)
% wrapped funciton of autogen_hoppingDyanmics
% INPUTS:
%   t= [1,n] = time line for states
%   z= [6,n] = [pcx;pcy;sita;dpcx;dpcy;dsita] = state of the system
%   uEx= [4,n] = [pex;pey;fx;fy] = supporting point and forces
%   p= parameter struct
%      .g = gravity
%      .m = mass of robot
%      .I = inetia of the robot
% OUTPUTS:
% dz= dz/dt = time derivative of the states
pcx=z(1,:);
pcy=z(2,:);
sita=z(3,:);
dpcx=z(4,:);
dpcy=z(5,:);
dsita=z(6,:);
pex=uEx(1,:);
pey=uEx(2,:);
fx=uEx(3,:);
fy=uEx(4,:);
m=p.m;
g=p.g;
I=p.I;

[ddpcx,ddpcy,ddsita]=autoGen_hoppingDynamics(pcx,pcy,sita,dpcx,dpcy,dsita,pex,pey,fx,fy,m,g,I);

dz=[dpcx;dpcy;dsita;ddpcx;ddpcy;ddsita];