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

<<<<<<< HEAD
duration = 1; 
=======
duration = .9; 
>>>>>>> 520a83a50ca31b3721d9ff6bbc70e636f7cb39b2

while(true)
  pause(duration);
  ranges = robot.laser.LatestMessage.Ranges;
  
  for i = 1:360
      [x, y, th] = irToXy(i, ranges(i));
      xVals(i) = x;
      yVals(i) = y;
      bearings(i) = th;
  end
  
  minR = 1000; %minimum radial distance. Initially set to very large value
  mindx = 0; 
  mindy = 0;
<<<<<<< HEAD
  mindth = 0;
=======
>>>>>>> 520a83a50ca31b3721d9ff6bbc70e636f7cb39b2
  
  for j = 1:360
      xVal = xVals(j);
      yVal = yVals(j);
      br = bearings(j);
      %use Pythagorean Theorem to calculate the nearest distance in METERS
      r = sqrt(xVal^2 + yVal^2);
      if ((r > minObjectRange) && (r < maxObjectRange) ...
          && (abs(br) < maxBearing) && r < minR)
          minR = r;
          mindx = xVal;
          mindy = yVal;
<<<<<<< HEAD
          mindth = br;
      end
  end
  
  %disp(minR) 
  %plot(xVals, yVals, '*');
  plot(-mindy, mindx, 'X');
  axis([-2, 2, -2, 2]);
  title('Location of Nearest Object');
  ylabel('Distance in Front of Robot (m)');
  xlabel('Distance to Side of Robot (m)');
  
  %Distance servo part
  W = .08;
  Kp = 4.2;
  stillTurn = 0.2;
  velocity = minR - idealObjectRange;
  if (abs(velocity) < 0.01)
      velocity = 0;
  end
  kappa = Kp*mindth;
  w = kappa*max([stillTurn, velocity]);
  v_left = velocity + (W/2)*w;
  v_right = velocity - (W/2)*w;
  
  robot.sendVelocity(v_left, v_right);
=======
      end
  end
  
  disp(minR) 
  %plot(xVals, yVals, '*');
  plot([0 mindx], [0 mindy], '-r*');
  axis([-1, 1, -1, 1]);
  
  %Distance servo part
  W = .08;
  velocity = minR - idealObjectRange;
  kappa = mindy/minR^2;
  w = kappa*velocity;
  v_left = velocity + (W/2)*w;
  v_right = velocity - (W/2)*w;
  
  robot.sendVelocity(v_right, v_left);
>>>>>>> 520a83a50ca31b3721d9ff6bbc70e636f7cb39b2
   
end






