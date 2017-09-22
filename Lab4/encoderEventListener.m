function encoderEventListener(handle, event)

% Note: this is currently the unchanged Lab 3 code.

global pTime;
global timestamp;
global leftPrev;
global rightPrev;
global v_left;
global v_right;
global dt;
global encI;
global encDT;

left = event.Vector.X;
right = event.Vector.Y;
timestamp = double(event.Header.Stamp.Sec) + ...
 double(event.Header.Stamp.Nsec)/1000000000.0;

ds_left = left - leftPrev;
ds_right = right - rightPrev;
dt = timestamp - pTime;
encDT(encI) = dt;
v_left(encI) = ds_left/dt;
v_right(encI) = ds_right/dt;

[X_R, Y_R, TH_R] = modelDiffSteerRobot(v_left, v_right, 0, 10, encDT);
plot(X_R, Y_R);
%xlim([-1 0.5]);
%ylim([-0.5 0.5]);
title('Location of Robot');
xlabel('X Position');
ylabel('Y Position');

% After calculations, update
leftPrev = event.Vector.X;
rightPrev = event.Vector.Y;
pTime = timestamp;

encI = encI + 1;
end