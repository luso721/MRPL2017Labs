%% Lab 1 Task 1: Move the Robot
robot = raspbot();

%Make the robot move forward 20cm
start_dist = robot.encoders.LatestMessage.Vector.X;
while(true)
    current_dist = robot.encoders.LatestMessage.Vector.X;
    %Magnitude of distace travelled
    if(abs(current_dist - start_dist) >= .2)
        break
    end
    robot.sendVelocity(.05,.05);
end
robot.stop();

pause(.05);

%Make the robot move back 20 cm
start_dist = robot.encoders.LatestMessage.Vector.X;
while(true)
    current_dist = robot.encoders.LatestMessage.Vector.X;
    if(abs(current_dist - start_dist) >= .2)
        break
    end
    robot.sendVelocity(-.05,-.05);
end
%Robot will run last action in loop for an extra second, so forcibly stop
%it.
robot.stop();

%% Task 2,3,4 
clear all;
close all; 
clc; 
robot = raspbot();

figure('Name', 'Robot Encoder Movements');
hold on;
xlabel('Time (s)');
ylabel('Distance Traveled (cm)');
title('Robot Encoder Readings');
%robot.sendVelocity(0, 0);

leftStart = robot.encoders.LatestMessage.Vector.Y;
rghtStart = robot.encoders.LatestMessage.Vector.X;
leftEncoder = leftStart;
rghtEncoder = rghtStart;
signedDistance = 0;

timeArray = zeros(1, 1);
leftArray = zeros(1, 1);
rghtArray = zeros(1, 1);
realLeft = zeros(1, 1);
realRght = zeros(1, 1);
v = 50;
dt = 0.05;

i = 2;
tic;
ptoc = toc;
travelDistance = 20; %cm


while (signedDistance < travelDistance)
    robot.sendVelocity(v/1000, v/1000);
    pause(dt); 
    ctoc = toc;
    leftEncoder = leftEncoder + v*(ctoc-ptoc); %robot.encoders.LatestMessage.Vector.Y;
    rghtEncoder = rghtEncoder + v*(ctoc-ptoc); %robot.encoders.LatestMessage.Vector.X;
    leftArray(i) = leftEncoder - leftStart;
    rghtArray(i) = rghtEncoder - rghtStart;
    timeArray(i) = toc;
    %leftEncoder = v*toc;
    %rghtEncoder = v*toc;
    realLeft(i) = (robot.encoders.LatestMessage.Vector.Y - leftStart)*100;
	realRght(i) = (robot.encoders.LatestMessage.Vector.X - rghtStart)*100;
    signedDistance = mean([realLeft(i), realRght(i)]); %mean([leftEncoder, rghtEncoder]);
    %plot(timeArray, leftArray, timeArray, rghtArray, timeArray, realLeft, timeArray, realRght);
    hold on;
    plot(timeArray, realLeft, timeArray, realRght)
    ptoc = ctoc;
    i = i + 1; 
end

robot.stop();
pause(1);
v = -1 * v;
signedDistance = 20;

while (signedDistance > 0)
    robot.sendVelocity(v/1000, v/1000);
    pause(dt); 
    ctoc = toc;
    leftEncoder = leftEncoder + v*(ctoc-ptoc); %robot.encoders.LatestMessage.Vector.Y;
    rghtEncoder = rghtEncoder + v*(ctoc-ptoc); %robot.encoders.LatestMessage.Vector.X;
    leftArray(i) = leftEncoder - leftStart;
    rghtArray(i) = rghtEncoder - rghtStart;
    timeArray(i) = toc;
    %leftEncoder = v*toc;
    %rghtEncoder = v*toc;
    realLeft(i) = (robot.encoders.LatestMessage.Vector.Y - leftStart)*100;
	realRght(i) = (robot.encoders.LatestMessage.Vector.X - rghtStart)*100;
    signedDistance = mean([realLeft(i), realRght(i)]); %mean([leftEncoder, rghtEncoder]);
    %plot(timeArray, leftArray, timeArray, rghtArray, timeArray, realLeft, timeArray, realRght);
    hold on;
    plot(timeArray, realLeft, timeArray, realRght)
    ptoc = ctoc;
    i = i + 1; 
end
robot.stop();
legend('Left Encoder', 'Right Encoder');
