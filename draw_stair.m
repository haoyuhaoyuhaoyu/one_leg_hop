function draw_stair(stepLength, stepHeight, step_Num)
    k = step_Num-1;
    p0 = [stepLength/2 + stepLength*k, 0 + stepHeight*k];
    p1 = [stepLength/2 + stepLength*k, stepHeight + stepHeight*k];
    p2 = [stepLength*3/2 + stepLength*k, stepHeight + stepHeight*k];
    p3 = [stepLength*3/2 + stepLength*k, 0 + stepHeight*k];
    pLeft = [-5*stepLength + stepLength*k, stepHeight*k];
    pRight = [5*stepLength + stepLength*k, stepHeight*k];
    if step_Num > 1
        x = [p0(1), p1(1), p2(1)];
        y = [p0(2), p1(2), p2(2)];
    else
        x = [p2(1), p1(1), p0(1), pLeft(1), pRight(1)];
        y = [p2(2), p1(2), p0(2), pLeft(2), pRight(2)];
    end
    plot(x,y,'LineWidth',1.5,'color','k');
    hold on
end