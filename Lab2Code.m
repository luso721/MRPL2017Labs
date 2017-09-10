clear all;
clc; 

%%Test 1
robot = raspbot();
robot.startLaser();
xVals = zeros(1,360);
yVals = zeros(1,360);
bearings = zeros(1,360);

minObjectRange = .06;
maxObjectRange = 1.5;
idealObjectRange = 0.5;
maxBearing = pi/2;

figure(1)

while(true)
  pause(1);
  ranges = robot.laser.LatestMessage.Ranges;
  
  for i = 1:360
      [x, y, th] = irToXy(i, ranges(i));
      xVals(i) = x;
      yVals(i) = y;
      bearings(i) = th;
  end
  
  minR = 100; %minimum radial distance
  
  for j = 1:360
      xVal = xVals(j);
      yVal = yVals(j);
      br = bearings(j);
      r = sqrt(xVal^2 + yVal^2);
      if ((r > minObjectRange) && (r < maxObjectRange) ...
          && (abs(br) < maxBearing) && r < minR)
          minR = r;
          mindx = xVal;
          mindy = yVal;
      end
  end
  
  %use Pythagorean Theorem to calculate the nearest distance in METERS
  disp(minR) 
  %plot(xVals, yVals, '*');
  plot([0 mindx], [0 mindy], '-r*');
  axis([-1, 1, -1, 1]);
  
  %Distance servo part
  
  objectposX = mindx;
  objectposY = mindy;
  
  %if (minR < idealObjectRange)
  %    backVelocity = objectRange - idealObjectRange;
  %end
  
  
  
  
end






