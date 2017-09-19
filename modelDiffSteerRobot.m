function [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)
    
    W = 0.08;
    V = (vr + vl) / 2;
    w = (vr - vl) / W;
    
    x = zeros(1);
    y = zeros(1);
    th = zeros(1);
    
    for i = 2:size(vr, 2)
        th(i) = th(i-1) + w(i)*dt/2;
        x(i) = x(i-1) + V(i)*cos(th(i))*dt;
        y(i) = y(i-1) + V(i)*sin(th(i))*dt;
        th(i) = th(i-1) + w(i)*dt;
    end
    
    % figure;
    % plot(x, y);
    
end