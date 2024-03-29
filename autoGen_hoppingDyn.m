function [ddpcx,ddpcy,ddsita] = autoGen_hoppingDyn(pcx,pcy,sita,dpcx,dpcy,dsita,pex,pey,fx,fy,m,g,I)
%AUTOGEN_HOPPINGDYN
%    [DDPCX,DDPCY,DDSITA] = AUTOGEN_HOPPINGDYN(PCX,PCY,SITA,DPCX,DPCY,DSITA,PEX,PEY,FX,FY,M,G,I)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    02-Feb-2021 14:32:00

t2 = 1.0./m;
ddpcx = fx.*t2;
if nargout > 1
    ddpcy = t2.*(fy-g.*m);
end
if nargout > 2
    ddsita = (fx.*pcy-fy.*pcx-fx.*pey+fy.*pex)./I;
end
