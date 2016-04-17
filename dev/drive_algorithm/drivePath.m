function drivePath
    syms t;
    n = 5;
    resolution = 0.01;

    
    %the math
    
    %path to drive
    x = t;
    y = sin(t);
    
    speed = 0.01;
    Px(1) = 0;
    Py(1) = 0;
    
    %tangent for heading grad(f)
    Dx = diff(x);
    Dy = diff(y);
    %heading = atan2(Dy, Dx);
    
    %the robot motion
    Kph = 0.05;
    Kpd = 0;
    H(1) = pi/4;
    
    format long;
    for i=2:n/resolution
        %msg = [i, resolution, i*resolution, fix(i*resolution)]
        j = i;
        k = i - 1;
        %desired angle x/y components
        Ax(j) = double(subs(Dx, t, i/resolution));
        Ay(j) = double(subs(Dy, t, i/resolution));
        %desired angle
        Hd = atan2(Ay(j), Ax(j));
        %current angle
        Hc = H(k);
        H(j) = H(k) - Kph*(H(k) - atan2(Ay(j), Ax(j)) );
        H(j);
        %robot position
        Px(j) = Px(k) + speed*cos(H(j));
        Py(j) = Py(k) + speed*sin(H(j));
    end
    H(30:40);
    atan2(Ay(30:40), Ax(30:40));
    
    %the plots
    figure(1)
    s = 0:resolution:n;
    plot(subs(x,t,s), subs(y,t,s), 'r');
    hold on;
    plot(Px, Py);
    s = 0:10*resolution:n;
    %t1 = vpa(cos(subs(heading, t, s)));
    %t2 = vpa(sin(subs(heading, t, s)));
    %quiver(vpa(subs(x,t,s)),vpa(subs(y,t,s)),t1,t2);
    hold off;
end