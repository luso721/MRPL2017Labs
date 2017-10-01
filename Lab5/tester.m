clear classes;
clear all;
close all;
clc;

robot = raspbot();
robot.sendVelocity(0, 0);
pause(0.05);

ref = figure8ReferenceControl(3, 1, 0.5);
traj = robotTrajectory([0, 0, 0], 0, ref);
signal = trajectoryFollower(robotModel, traj);

x = zeros(1);
y = zeros(1);
figure;

firstIteration = false;
T = 0;
i = 1;

while (T < ref.T_f + 2*ref.t_pause)
    if (firstIteration == false)
        timer = tic;
        firstIteration = true;
    end
    T = toc(timer);
    [vl, vr] = getvlvrAtTime(signal, robotModel, traj, T);
    robot.sendVelocity(vl, vr);
    pose = getPoseAtTime(traj, T);
    x(i) = pose(1);
    y(i) = pose(2);
    plot(x, y);
    i = i + 1;
    pause(0.05);
end

robot.stop();