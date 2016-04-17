function [distance, direction, done] = driveOnPath(path_x, path_y, pos_x, pos_y)
    %follower settings
    max_angle = pi/4;
    max_distance = 4000; %encoder ticks away from path
    distance_gain = max_angle / max_distance;
    max_speed = 0.5;
    
    %get desired
    [distance_from_path, tangent] = closest_point(path_x, path_y, pos_x, pos_y);
    
    %calculate output
    %direction we want to travel,
    distance_correction = distance_gain * distance_from_path;
    if (abs(distance_correction) > max_angle)
        distance_correction = max_angle * distance_correction/abs(distance_correction);
    end
    
    direction = tangent + distance_correction;
    distance = max_speed * 1;
    
    %is done?
    if 0
        done = 1;
    else
        done = 0;
    end
end

function [distance_from_path,angle_of_path] = closest_point(path_x, path_y, pos_x, pos_y)
    syms t;
    i = 0:5/100:7;
   
    plot(path_x(i), path_y(i));

    Tx = diff(path_x, t);
    Ty = diff(path_y, t);
    Nx = -Ty;
    Ny = Tx;
    Nm = Ny/Nx;


    x2 = 0:5;
 
    %m = subs(Nx, t, i)/subs(Ny, t, i)
   
    %plot start point
    scatter(pos_x, pos_y, 'r', 'Linewidth', 1);
    
    min = [1000000, 0];
    result = 0;
   
    for i=0:0.1:5
        
        result = vpa(subs(Nm, t, i)*(pos_x-path_x(i))) + path_y(i) - pos_y;
        tic
        if abs(result) < min(1)
            min = [abs(result), i];
        end
        toc
    end

    y2 = subs(Nm, t, min(2))*(x2 - path_x(min(2))) + path_y(min(2));
    plot(x2,y2, 'r', 'Linewidth', 1);
    axis equal;
    axis([0 7 -0.1 2]);

    distance_from_path = sqrt((path_x(min(2)) - pos_x)^2 + (path_y(min(2))-pos_y)^2);
    angle_of_path = atan2(vpa(subs(Ty, t,min(2))), vpa(subs(Tx, t,min(2))));

    quiver(path_x(min(2)), path_y(min(2)), subs(Tx, t, min(2)), subs(Ty, t, min(2)));
    
end