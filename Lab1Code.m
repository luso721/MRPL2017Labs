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
robot = raspbot();

%robot.sendVelocity(0, 0);

leftStart = robot.encoders.LatestMessage.Vector.Y;
rghtStart = robot.encoders.LatestMessage.Vector.X;
%leftStart = rand*1000;
%rghtStart = rand*1000;
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
totalDistance = 0;
while (signedDistance < 304.8)
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
    realLeft(i) = (robot.encoders.LatestMessage.Vector.Y - leftStart)*1000;
	realRght(i) = (robot.encoders.LatestMessage.Vector.X - rghtStart)*1000;
    signedDistance = mean([realLeft(i), realRght(i)]); %mean([leftEncoder, rghtEncoder]);
    %plot(timeArray, leftArray, timeArray, rghtArray, timeArray, realLeft, timeArray, realRght);
    plot(timeArray, realLeft, timeArray, realRght)
    ptoc = ctoc;
    i = i + 1; 
end

v = -1 * v;

while (signedDistance >= 0)
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
    realLeft(i) = (robot.encoders.LatestMessage.Vector.Y - leftStart)*1000;
	realRght(i) = (robot.encoders.LatestMessage.Vector.X - rghtStart)*1000;
    signedDistance = mean([realLeft(i), realRght(i)]); %mean([leftEncoder, rghtEncoder]);
    %plot(timeArray, leftArray, timeArray, rghtArray, timeArray, realLeft, timeArray, realRght);
    plot(timeArray, realLeft, timeArray, realRght)
    ptoc = ctoc;
    i = i + 1; 
end

robot.stop();