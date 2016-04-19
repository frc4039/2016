function [x,y] = updatePos(lastx, lasty, distance, angle)
    x = lastx + distance * cos(angle);
    y = lasty + distance * sin(angle);
end
