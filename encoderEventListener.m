function encoderEventListener(handle, event)

global pTime;
global timestamp;
global leftPrev;
global rightPrev;
global ds_left;
global ds_right;
global dt;

left = event.Vector.X;
right = event.Vector.Y;
timestamp = double(event.Header.Stamp.Sec) + ...
 double(event.Header.Stamp.Nsec)/1000000000.0;

ds_left = left - leftPrev;
ds_right = right - rightPrev;
dt = timestamp - pTime;

% After calculations, update
leftPrev = event.Vector.X;
rightPrev = event.Vector.Y;
pTime = timestamp;

end