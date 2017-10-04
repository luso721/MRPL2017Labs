clear classes;
clear all;
close all;
clc;

robot = raspbot();
robot.encoders.NewMessageFcn=@encoderEventListener;
robot.sendVelocity(0, 0);
pause(0.05);

global encI;
global T_R;
global X_R;
global Y_R;
global TH_R;
global leftPrev;
global rightPrev;
global timePrev;
global startTime;
global VL;
global VR;

leftPrev = robot.encoders.LatestMessage.Vector.X;
rightPrev = robot.encoders.LatestMessage.Vector.Y;
timePrev = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + ...
 double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000.0;
startTime = timePrev;

T_R = zeros(1);
X_R = zeros(1);
Y_R = zeros(1);
TH_R = zeros(1);

ref = figure8ReferenceControl(3, 1, 0.5);
traj = robotTrajectory([0, 0, 0], 0, ref);
signal = trajectoryFollower(robotModel, traj);

t = zeros(1);
x = zeros(1);
y = zeros(1);
th = zeros(1);
x_err = zeros(1);
y_err = zeros(1);
th_err = zeros(1);
figure(1);
figure(2);
figure(3);

firstIteration = false;
T = 0;
i = 1;
encI = 1;
delay = 0.3;

ffw = 1;
fbk = 1;

while (T < getTrajectoryDuration(ref)+delay+1)
    if (firstIteration == false)
        firstIteration = true;
        pause(0.05);
        timer = tic;
        encI = 1;
        continue;
    end
    T = toc(timer);
    %[vl, vr] = getvlvrAtTime(signal, robotModel, traj, T);
    %robot.sendVelocity(vl, vr);
    ref_pose = getPoseAtTime(traj, T-delay);
    t(i) = T;
    x(i) = ref_pose(1);
    y(i) = ref_pose(2);
    th(i) = ref_pose(3);
    
    %robot pose
    x_rob = X_R(end);
    y_rob = Y_R(end);
    th_rob = TH_R(end);
    rob_pose = [x_rob; y_rob; th_rob];
    [V_rob, w_rob] = robotModel.vlvrToVw(VL, VR);
    
    [V_ref, w_ref] = computeControl(ref, T-delay);
    
    %find errors
    control = controller(rob_pose, ref_pose, V_rob);
    [r_err, e_theta] = computeError(control);
    x_err(i) = r_err(1);
    y_err(i) = r_err(2);
    th_err(i) = e_theta;
    
    %compute feedforward velocities
    [vl, vr] = getvlvrAtTime(signal, robotModel, traj, T);

    %compute feedback velocities
    [V_fdbk, w_fdbk] = computeFeedback(control);
    [vl_fdbk, vr_fdbk] = robotModel.VwTovlvr(V_fdbk, w_fdbk);
    
    %apply corrected velocity to robot
    vl_tot = ffw*(vl) + fbk*(vl_fdbk);
    vr_tot = ffw*(vr) + fbk*(vr_fdbk);
    
    if ((vl_tot < 0) || (vl_tot >= 0))
    else
        vl_tot = 0;
    end
    if ((vr_tot < 0) || (vr_tot >= 0))
    else
        vr_tot = 0;
    end
    
    robot.sendVelocity(vl_tot, vr_tot);
    
    i = i + 1;
    pause(0.05);
end

robot.stop();
robot.encoders.NewMessageFcn=[];


    figure(1);
    hold on;
    %yyaxis left
    plot(t, x, '-b', T_R, X_R, '-r');
    plot(t, y, '--b', T_R, Y_R, '--r');
    %yyaxis right
    plot(t, th, ':b', T_R, TH_R, ':r');
    hold off;
    figure(2);
    hold on;
    plot(x, y, '-b');
    plot(X_R, Y_R, '-r');
    hold off;
    figure(3);
    hold on;
    %yyaxis left
    plot(t, y_err, '-r');
    plot(t, x_err, '--r');
    %yyaxis right
    plot(t, th_err, ':r');
    hold off;
    

figure(1);
title('Position vs Time');
xlabel('Time (s)');
%yyaxis left
ylabel('Position');
%yyaxis right
%ylabel('Theta (rad)');
legend('Location', 'NE', 'X Simulation', 'X Encoder Data', 'Y Simulation', ...
    'Y Encoder Data', 'Theta Simulation', 'Theta Encoder Data');
figure(2);
title('Location of Robot');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Location', 'NW', 'Simulation', 'Encoder Data');
figure(3);
title('Error');
xlabel('Time (s)');
%yyaxis left
ylabel('Error');
%yyaxis right
%ylabel('Heading Error (rad)');
legend('Location', 'NW', 'Crosstrack Error', 'Alongtrack Error', 'Heading Error');