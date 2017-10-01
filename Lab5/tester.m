clear classes;
clear all;
close all;
clc;

ref = figure8ReferenceControl(3, 1, 0.5);
traj = robotTrajectory([0, 0, 0], 0, ref);

figure;
hold on;
plot(traj.x, traj.y);

for t = 0:0.5:18
    p1 = getPoseAtTime(traj, t + 0.000053234);
    plot(p1(1), p1(2), 'or');
end