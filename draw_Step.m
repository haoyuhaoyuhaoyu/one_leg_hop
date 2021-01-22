function draw_Step(soln, p, step_Num, stair_Num)
    k = step_Num-1;
    state = soln.grid.state;
    control = soln.grid.control;

    pcx = state(1,:);
    pcy = state(2,:);
    sita = state(3,:);
    pex = control(1,:);
    pey = control(2,:);
    % add step offset
    pcx = pcx + p.stepLength*k;
    pcy = pcy + p.stepHeight*k;
    pex = pex + p.stepLength*k;
    pey = pey + p.stepHeight*k;
    
    phase_separate = fix(p.user_grid/2);
    % stair constr
    pC = [p.stepLength/2 + p.stepLength*k, p.stepHeight*1.3 + p.stepHeight*k];
    % swing phase start
    p0 = [pex(phase_separate), pey(phase_separate)];
    % swing phase end
    pF = [pex(p.user_grid), pey(p.user_grid)];
    % interp
    [x,y] = interp(p0,pF,pC,phase_separate);
    pex(phase_separate+1:end) = x;
    pey(phase_separate+1:end) = y;
    for i=1:1:60
        hold off
        plot([pcx(i),pex(i)],[pcy(i),pey(i)],'LineWidth',2);
        hold on
        for j = 1:1:stair_Num
            draw_stair(p.stepLength, p.stepHeight, j);
            hold on
        end
        draw_robot(sita(i), pcx(i),pcy(i));
        hold on
        plot(pcx(i),pcy(i),'o',...
            'MarkerSize',5);
        axis equal
        xlim([-2+pcx(i),2+pcx(i)]);
        ylim([-1+pcy(i),2.5+pcy(i)]);  
        hold on
        drawnow;
        pause(0.03);
    end
end