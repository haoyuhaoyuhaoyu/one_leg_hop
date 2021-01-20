function [check_pathC, check_pathCeq, check_stepC, check_stepCeq,...
    check_Dyn] = check_Constr(soln, P)    
    x = soln.grid.state;
    u = soln.grid.control;
    [~, Num] = size(x);
    x0 = x(:,1);
    xF = x(:,Num);
    uF = u(:,Num);
    x_pre = x(:,1:Num-1);
    x_k = x(:, 2:Num);
    dt = P.stepTime/P.user_grid;
    disp(xF)
    [c1, ceq1] = pathConstraint(xF, uF);
    [c2, ceq2] = stepConstraint(x0, xF, P);
    check_pathC = c1;
    check_pathCeq = ceq1;
    check_stepC = c2;
    check_stepCeq = ceq2;
    
    check_Dyn = zeros(Num-1,1);
    for i = 1:1:Num-1
       check_Dyn(i)= norm(x_k(1:3,i)-x_pre(1:3,i) - 0.5*dt*(x_k(4:6,i)+x_pre(4:6,i)));
    end
end