% Note: probably use timestamp to better calculate dt
% Note: this assumes left and right wheel behave the same
    % Could be made more robust by calculating them separately

close all;
clear all;
clc;

robot = raspbot();
robot.sendVelocity(0, 0);
pause(0.05);

% Set starting values and PID constants
leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;
x_goal = 0.1;
Kp = 10.0;
Kd = 0.0;
Ki = 0.0;
x = 0;
error = x_goal - x;
error_prev = error;
i_error = 0;
maxI_error = 1;
maxV = 0.3;
tolerance = 0.0001;
timeLimit = 6;
dt = 0.05;

% Begin timer
timer = tic;
T = toc(timer);
T_prev = T;

% Stop when tolerance or time limit is reached
while((abs(error) >= tolerance) && (T < timeLimit))
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rightEncoder = robot.encoders.LatestMessage.Vector.Y;
    
    % Find current position and errors
    x = mean([leftEncoder-leftStart, rightEncoder-rightStart]);
    error = x_goal - x;
    d_error = (error - error_prev) / dt;
    i_error = i_error + error * dt;
    
    % Clamp i_error
    i_error = sign(i_error) * min([abs(i_error), maxI_error]);
    
    % Calculate control
    v = Kp*error + Kd*d_error + Ki*i_error;
    v = sign(v) * min([abs(v), maxV]);
    
    % Finally, send the velocity
    robot.sendVelocity(v, v);
    
    % Update time and previous values;
    error_prev = error;
    T_prev = T;
    T = toc(timer);
    pause(dt);
end

robot.stop();