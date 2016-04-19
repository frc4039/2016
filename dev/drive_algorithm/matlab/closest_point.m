function closest_point
syms t;
i = 0:5/100:7;
x = @(t) t;
y = @(t) sin(t) + t/5;

plot(x(i), y(i));
hold on;

Tx = diff(x, t);
Ty = diff(y, t);
Nx = -Ty;
Ny = Tx;
Nm = Ny/Nx;


x2 = 0:5;
i = 3;
x0 = x(i);
y0 = y(i);
%m = subs(Nx, t, i)/subs(Ny, t, i)
Px = 1.7044;
Py = 0;
scatter(Px, Py, 'r', 'Linewidth', 1);
min = [1000000, 0];
for i=0:0.1:5
    
    result = vpa(subs(Nm, t, i)*(Px-x(i))) + y(i) - Py;

    if abs(result) < min(1)
        min = [abs(result), i];
    end
    
end
y2 = subs(Nm, t, min(2))*(x2 - x(min(2))) + y(min(2));
plot(x2,y2, 'r', 'Linewidth', 1);
axis equal;
axis([0 7 -0.1 2]);

distFromPath = sqrt((x(min(2)) - Px)^2 + (y(min(2))-Py)^2);
desHeading = atan2(vpa(subs(Ty, t,min(2))), vpa(subs(Tx, t,min(2))));

quiver(x(min(2)), y(min(2)), subs(Tx, t, min(2)), subs(Ty, t, min(2)));

hold off;