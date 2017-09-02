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
  
%% Task 2: Simulation   

left_start = 6934;
right_start = 4396;

%Since encoder readings are in mm, one "tick" of the encoder is 1mm

start_time = tic; 
final_dist = 304.8; % in mm
ticks_per_cm = 10;

while(true)
    left_encoder = 6934 + 5*10; 
    right_encoder = 4398 + 5*10;
    signed_distance = ((left_encoder - left_start))/1000 + ((right_encoder-right_start))/1000;
    signed_distance = signed_distance/2;
    if (signed_distance >= final_dist) 
        break
    end
    pause(0.001);
    
end






%% Task 4: Basic Plotting and Real Time Plotting