
clear all;
clc; 

%%Test 1
robot = raspbot();
robot.startLaser();
xVals = zeros(1,360);
yVals = zeros(1,360);
figure(1);

while(true)
  pause(1);
  ranges = robot.laser.LatestMessage.Ranges; 
 
  
  for i = 1:360
      [x, y, th] = irToXy(i, ranges(i));
      xVals(i) = x;
      yVals(i) = y;
  end
  
  plot(xVals, yVals, '*');
  axis([-5, 5, -5, 5]);
  
end






