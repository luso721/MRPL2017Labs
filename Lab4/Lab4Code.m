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

timer = tic;
T = toc(timer);
T_prev = T;

t_ramp = vmax/amax;
time_limit = 5;
t_f = (1 + (vmax^2/amax))/vmax;

ref_dist = zeros(1);
times = zeros(1);
actual_dist = zeros(1);
velocities = zeros(1);

index = 1;
dist_prev = 0;
dt = .001;
sgn = 1;

%robot position and refrence position are out of phase by t = delay
%so synchronize them. value determined experimentally 
delay = .3;

while(T < t_f )   
    %compute distance travelled in simulation
    simulated_dist = mean([leftEncoderSim, rightEncoderSim]);
    
    %compute distance travelled by robot
    leftEncoder = robot.encoders.LatestMessage.Vector.X;
    rightEncoder = robot.encoders.LatestMessage.Vector.Y;
    rob_dist = mean([leftEncoder-leftStart, rightEncoder - rightStart]);
    
    %feedforward error will be difference of simulated distance and 
    %traveled distance (not done yet)
    %err = abs(goal_dist - current_dist);
    
    sgn = sign(-rob_dist + goal_dist);
    %move the robot
    times(index) = T;
    v_ref = trapezoidalVelocityProfile(T, amax, vmax, goal_dist, sgn);
    
    v_delay = trapezoidalVelocityProfile(T-delay, amax, vmax, goal_dist, sgn);
    
    %integrate
    if (T > delay)
       dist_travelled = dist_prev + v_delay * (T - T_prev);
    
    else
        dist_travelled = dist_prev + 0 * (T - T_prev);
    end
    
    %assign previous distances and array indices
    dist_prev = dist_travelled;
    ref_dist(index) = dist_travelled;
    actual_dist(index) = rob_dist;
    velocities(index) = v_ref;
    
    leftEncoderSim = leftEncoderSim + v_ref*dt;
    rightEncoderSim = rightEncoderSim + v_ref*dt;
    
    robot.sendVelocity(v_ref, v_ref);
    
    index = index + 1;
    T_prev = T;
    T = toc(timer);
    pause(dt);
    
    plot(times, velocities, times, ref_dist, times, actual_dist)
end

%plot(times, velocities);
%plot(times, actual_dist);
%plot(times, velocities, times, ref_dist, times, actual_dist)
xlabel('time (s)');
ylabel('position (m)');
legend('Velocity Profile', 'Synchronized Refrence Distance','Robot Distance');
















