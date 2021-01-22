function draw_MultiStep(soln, p, step_Num)
    stair_Num = step_Num + 2;
    for i = 1:1:step_Num
        draw_Step(soln, p, i, stair_Num);
    end
end