% Note: probably use timestamp to better calculate dt
% Note: this assumes left and right wheel behave the same
% Could be made more robust by calculating them separately

close all;
clear all;
clc;

%% Code for actual robot
%robot = raspbot();
%robot.sendVelocity(0, 0);
pause(0.05);

% Set starting values and PID constants
%leftStart = robot.encoders.LatestMessage.Vector.X;
%rightStart = robot.encoders.LatestMessage.Vector.Y;
x_goal = 0.1;
Kp = 7.0;
Kd = 0.4;
Ki = .0;
x = 0;
error = x_goal - x;
error_prev = error;
i_error = 0;
maxI_error = 1;
maxV = 0.3;
tolerance = 0.01;
timeLimit = 6;
dt = 0.5;

% Begin timer
timer = tic;
T = toc(timer);
T_prev = T;
index = 1;

time = zeros(1);
error_vals = zeros(1);


while((abs(error) >= tolerance) && (T < timeLimit))
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rightEncoder = robot.encoders.LatestMessage.Vector.Y;
    
    % Find current position and errors
    x = mean([leftEncoder-leftStart, rightEncoder-rightStart]);
    error = x_goal - x;
    error_vals(index) = error;
    time(index) = T;
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

plot(time, error_vals);
robot.stop();

%% Code for Simulation/Feedforward Control on Robot/ChallengeTask
% simulates the ramp function (3.32)


close all;
clear all;
clc;

global dsleft
global dsright

robot = raspbot();
robot.sendVelocity(0, 0);
pause(0.05);

leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;

%Ramp function parameters
vmax = .25;
amax = 3*0.25;
goal_dist = 1; %meters
current_dist = 0;
err = goal_dist - current_dist;

leftEncoderSim = 0;
rightEncoderSim = 0;

t_ramp = vmax/amax;
time_limit = 5;
t_f = (goal_dist + (vmax^2/amax))/vmax;

ref_dist = zeros(1);
times = zeros(1);
actual_dist = zeros(1);
velocities = zeros(1);

index = 1;
dist_travelled = 0;
error_prev = 0;
i_error = 0;
dt = 0.02;
sgn = 1;
t_f = t_f - 0.117;

figure(1);
figure(2);

fbk = 0;

%PID parameters/Good values for robot 5
Kp = 7.2;%7.8;  %7.2
Kd = 0.15;%0.15;
Ki = 0.05;%0.05;    %0.05
maxI_error = 0.5;%0.05;
encTuner = 0.9765;%0.9908;

%robot position and refrence position are out of phase by t = delay
%so synchronize them. value determined experimentally 
delay = 0.25; % 0.3 good

timer = tic;
T = toc(timer);
T_prev = T;

while(T < t_f + 1)  
    
    if (index == 2)
        timer = tic;
        T = toc(timer);
        ptoc = T;
    end
    T = toc(timer);
    
    %compute distance travelled in simulation
    simulated_dist = mean([leftEncoderSim, rightEncoderSim]);
    
    %compute distance travelled by robot
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rightEncoder = robot.encoders.LatestMessage.Vector.Y;
    rob_dist = encTuner * mean([leftEncoder-leftStart, rightEncoder - rightStart]);
    
    %feedforward error will be difference of simulated distance and 
    %traveled distance (not done yet)
    %err = abs(goal_dist - current_dist);
    
    sgn = sign(-rob_dist + goal_dist);
    sgn_sim = sign(-dist_travelled + goal_dist);
    %move the robot
    times(index) = T;
    v_ref = trapezoidalVelocityProfile(T, amax, vmax, goal_dist, sgn);
    v_delay = trapezoidalVelocityProfile(T-delay, amax, vmax, goal_dist, sgn_sim);
    v2(index) = v_delay;
    %integrate
    dist_travelled = dist_travelled + v_delay * (T - T_prev);
    if (T - delay > t_f)
        dist_travelled = 1.0;
    end
    
    % Calculate feedback
    error = dist_travelled - rob_dist;
    
    d_error = (error - error_prev) / (T - T_prev);
    i_error = i_error + error * (T - T_prev);
    
    i_error = sign(i_error) * min([abs(i_error), maxI_error]);
    
    v_pid = Kp*error + Kd*d_error + Ki*i_error;
    
    % Get final velocity
    v = v_ref + fbk*v_pid;
    % v = sign(v) * min([abs(v), vmax]);
    
    %assign previous distances and array indices
    error_prev = error;
    ref_dist(index) = dist_travelled;
    actual_dist(index) = rob_dist;
    velocities(index) = v_ref;
    
    leftEncoderSim = leftEncoderSim + v_ref*dt;
    rightEncoderSim = rightEncoderSim + v_ref*dt;
    
    robot.sendVelocity(v, v);
    
    index = index + 1;
    T_prev = T;
    pause(dt);
    
    figure(1);
    plot(times, ref_dist, times, actual_dist);
    figure(2);
    plot(times, actual_dist - ref_dist);
end

robot.stop();
finalError = -1*error(end)

%plot(times, velocities);
%plot(times, actual_dist);
%plot(times, velocities, times, ref_dist, times, actual_dist)
figure(1);
xlabel('Time (s)');
ylabel('Position (m)');
legend('Location', 'NW', 'Synchronized Reference Distance','Robot Distance');
figure(2);
xlabel('Time (s)');
ylabel('Error (m)');