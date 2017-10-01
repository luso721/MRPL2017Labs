clear classes;
clear all;
close all;
clc;

robot = raspbot();
robot.encoders.NewMessageFcn=@encoderEventListener;
robot.sendVelocity(0, 0);
pause(0.05);

global encI;
global encDT;
global T_R;
global X_R;
global Y_R;
global TH_R;
global leftPrev;
global rightPrev;
global timePrev;
global startTime;

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

x = zeros(1);
y = zeros(1);
figure(1);

firstIteration = false;
T = 0;
i = 1;
encI = 1;

while (T < ref.T_f + 2*ref.t_pause)
    if (firstIteration == false)
        timer = tic;
        encI = 1;
        firstIteration = true;
        continue;
    end
    T = toc(timer);
    [vl, vr] = getvlvrAtTime(signal, robotModel, traj, T);
    robot.sendVelocity(vl, vr);
    pose = getPoseAtTime(traj, T);
    x(i) = pose(1);
    y(i) = pose(2);
    
    figure(1);
    hold on;
    plot(x, y, '-b');
    plot(X_R, Y_R, '-r');
    hold off;
    
    i = i + 1;
    pause(0.02);
end

robot.stop();
robot.encoders.NewMessageFcn=[];

figure(1);
title('Location of Robot');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Location', 'NW', 'Simulation', 'Encoder Data');
