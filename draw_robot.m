function draw_robot(sita,pcx,pcy)
    L = 0.2;
    H = 0.1;
    Robot = [-L,-L,L,L;-H,H,H,-H];
    p_com = [pcx, pcy];
%     p0 = [p_com(1) - L, p_com(2) - H];
%     p1 = [p_com(1) - L, p_com(2) + H];
%     p2 = [p_com(1) + L, p_com(2) + H];
%     p3 = [p_com(1) + L, p_com(2) - H];
    R = [cos(sita), -sin(sita); sin(sita), cos(sita)];% p' = R*p
    Robot_R = R*Robot;
    P = [[p_com(1)+Robot_R(1,:),p_com(1)+Robot_R(1,1)];[p_com(2)+Robot_R(2,:),p_com(2)+Robot_R(2,1)]];
    plot(P(1,:),P(2,:),'LineWidth',1.5,'color','r');
end