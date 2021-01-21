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
    [c1, ceq1] = pathConstraint(x, u);
    [c2, ceq2] = stepConstraint(x0, xF, P);
    % check_pathC
    check_pathC(1:60,:) = [repmat({'c_MaxLL'},[60,1]),num2cell(c1(1:60))];
    check_pathC(61:120,:) = [repmat({'c_MinLL'},[60,1]),num2cell(c1(61:120))];
    check_pathC(121:150,:) = [repmat({'c_Cone'},[30,1]),num2cell(c1(121:150))];
    check_pathC(151:180,:) = [repmat({'c_SF'},[30,1]),num2cell(c1(151:180))];
    check_pathC(181,:) = [repmat({'c_SP'},[1,1]),num2cell(c1(181))];
    check_pathC(182:211,:) = [repmat({'c_SwingSPy'},[30,1]),num2cell(c1(182:211))];
    
    % check_pathCeq
    check_pathCeq(1:30,:) = [repmat({'ceq_SPy'},[30,1]),num2cell(ceq1(1:30))];
    check_pathCeq(31:60,:) = [repmat({'ceq_SPx'},[30,1]),num2cell(ceq1(31:60))];
    check_pathCeq(61:90,:) = [repmat({'ceq_SwFx'},[30,1]),num2cell(ceq1(61:90))];
    check_pathCeq(91:120,:) = [repmat({'ceq_SwFy'},[30,1]),num2cell(ceq1(91:120))];
    check_pathCeq(121,:) = [repmat({'ceq_SPx0'},[1,1]),num2cell(ceq1(121))];
    check_pathCeq(122,:) = [repmat({'ceq_LL'},[1,1]),num2cell(ceq1(122))];
    
    % check_stepC
    check_stepC(1,:) = [repmat({'c_vx0'},[1,1]),num2cell(c2(1))];
    check_stepC(2,:) = [repmat({'c_vy0'},[1,1]),num2cell(c2(2))];
    
    % check_stepCeq
    check_stepCeq(1,:) = [repmat({'ceq1'},[1,1]),num2cell(ceq2(1))];
    check_stepCeq(2,:) = [repmat({'ceq2'},[1,1]),num2cell(ceq2(2))];
    check_stepCeq(3,:) = [repmat({'ceq3'},[1,1]),num2cell(ceq2(3))];
%     check_stepCeq(4,:) = [repmat({'v_x'},[1,1]),num2cell(ceq2(4))];
%     check_stepCeq(5,:) = [repmat({'v_y'},[1,1]),num2cell(ceq2(5))];
%     check_stepCeq(6,:) = [repmat({'v_sita'},[1,1]),num2cell(ceq2(6))];
    
    check_Dyn = zeros(Num-1,1);
    for i = 1:1:Num-1
       check_Dyn(i)= norm(x_k(1:3,i)-x_pre(1:3,i) - 0.5*dt*(x_k(4:6,i)+x_pre(4:6,i)));
    end
end