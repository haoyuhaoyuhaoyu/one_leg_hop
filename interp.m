function [x,y] = interp(p0,pF,pC,num)
    ix = [p0(1),pC(1),pF(1)];
    iy = [p0(2),pC(2),pF(2)];
    p = polyfit(ix,iy,2); % p1*x^2 + p2*x + p3
    x = linspace(ix(1),ix(3),num);
    y = p(1)*x.^2 + p(2)*x + p(3);
end