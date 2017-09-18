close all;
clear all;
clc;

%robot = raspbot();
%robot.encoders.NewMessageFcn=@encoderEventListener;
%robot.sendVelocity(0, 0);
pause(0.05);

%leftStart = robot.encoders.LatestMessage.Vector.X;
%rightStart = robot.encoders.LatestMessage.Vector.Y;
global timestamp;
global leftPrev;
%leftPrev = leftStart;
global rightPrev;
%rightPrev = rightStart;
%iTime = timestamp;
global pTime;
%pTime = timestamp;
global ds;
global dt;
v = 0.2;
sf = 1;
tf = sf/v;
kth = 2*pi/sf;
kk = 15.1084;
ks = 3;
Tf = ks*tf;
notDt = 0.001;
t = 0:notDt:tf;
s = zeros(1);
kappa = zeros(1);
omega = zeros(1);
vr = zeros(1);
vl = zeros(1);
W = 0.08;

i = 1;
tic;
while(i <= size(t, 2))
    
    %robot.sendVelocity(vl(i), vr(i));
    %[x, y, th] = modelDiffSteerRobot(vl(i)*1000, vr(i)*1000, t(i, tf, notDt);
    %V(i) = ds/dt;
    %t(i) = timestamp - iTime;
    %plot(t, V);
    %ylim([0 0.1]);
    T = toc;
    t = T/ks;
    s(i) = v*t;
    kappa(i) = (kk/ks)*sin(kth*s(i));
    omega(i) = kappa(i)*v;
    vr(i) = v + W/2*omega(i) * 1000;
    vl(i) = v - W/2*omega(i) * 1000;
    pause(notDt);
    i = i + 1;
end

modelDiffSteerRobot(vl, vr, 0, tf, notDt);

%robot.stop();