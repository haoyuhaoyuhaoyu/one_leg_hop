function draw_stair(stepLength, stepHeight)
    p0 = [stepLength/2, 0];
    p1 = [stepLength/2, stepHeight];
    p2 = [stepLength*3/2, stepHeight];
    p3 = [stepLength*3/2, 0];
    pLeft = [-5*stepLength, 0];
    pRight = [5*stepLength, 0];
    x = [p0(1), p1(1), p2(1), p3(1), pLeft(1), pRight(1)];
    y = [p0(2), p1(2), p2(2), p3(2), pLeft(2), pRight(2)];
    plot(x,y,'LineWidth',1.5,'color','k');
end