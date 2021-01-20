function [opt_obj] = obj_torque(x, u)
%     [~, grid_num] = size(x);
%     leg_sita = zeros(grid_num,1);
%     pcx   = x(1,:);
%     pcy   = x(2,:);
%     pex   = u(1,:);
%     pey   = u(2,:);
%     for i=1:1:grid_num
%     %for stance phase constr
%         pc = [pcx(i), pcy(i)];
%         pe = [pex(i), pey(i)];
%         leg_sita(i) = atan((pc(1)-pe(1))/(pc(2)-pe(2)));
%     end
    sita = x(3,:);
    opt_obj = u(3,:).^2 + u(4,:).^2; % + 5*sita.^2;
    %opt_obj = u(3,:).^2 + u(4,:).^2;
end