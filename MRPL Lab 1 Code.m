%%Lab 1 Task 1 Move the Robot
robot = raspbot('Raspbot-5');
robot.sendVelocity(.05, .05);
pause(.5);
robot.sendVelocity(-.05, -.05);
pause(.5);
robot.stop();
