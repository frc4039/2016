function simulate
    %initialize variables
    hold on;
    x = 0;
    y = 0;
    i = 0;
    maxIterations = 10;
    done = 0;
    
    %set path to follow
    sym t
    path_x = @(t) t; 
    path_y = @(t) sin(t) + t/5;

    %simulation loop
    while done == 0 && i < maxIterations
        [robot_speed, robot_angle, done] = driveOnPath(path_x, path_y, x, y);
        [x,y] = updatePos(x,y,robot_speed, robot_angle);
        i = i + 1;
        display([num2str(i/maxIterations*100), '% complete']);
    end
    
    %results
    %Robot_Position = ['x: ', num2str(x), sprintf('\ty: '), num2str(y)];
    %display(Robot_Position)
    hold off;
end