close all;
clear all;
clc;

robot = raspbot();
robot.encoders.NewMessageFcn=@encoderEventListener;
robot.sendVelocity(0, 0);
%pause(0.05);

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
global ds_left;
global ds_right;
global dt;
v = 0.2;
sf = 1;
tf = sf/v;
kth = 2*pi/sf;
kk = 15.1084;
dT = 0.01;
ks = 3;
Tf = ks*tf;
s = zeros(1);
kappa = zeros(1);
omega = zeros(1);
vr = zeros(1);
vl = zeros(1);
W = 0.09;
global X;
global Y;
global TH;
X = zeros(1);
Y = zeros(1);
TH = zeros(1);

X_R = zeros(1);
Y_R = zeros(1);
TH_R = zeros(1);

DT = zeros(1);
correction = 0.005;

myPlot = plot(X, Y, 'b-');
xlim([-0.5 0.5]);
ylim([-0.5 0.5]);

i = 2;
tic;
T = toc;
ptoc = T;
while(T < 1.09*Tf)
    %[x, y, th] = modelDiffSteerRobot(vl(i)*1000, vr(i)*1000, t(i, tf, notDt);
    %V(i) = ds/dt;
    %t(i) = timestamp - iTime;
    %plot(t, V);
    %ylim([0 0.1]);
    T = toc;
    DT(i) = T - ptoc;
    %ks = (T - ptoc) / dt;
    t = T/ks;
    s(i) = v*t;
    kappa(i) = (kk/ks)*sin(kth*s(i));
    omega(i) = kappa(i)*v;
    if (T < 0.5*Tf) || (T > Tf)
        correction = 0.003;
    else
        correction = -0.003;
    end
    vr(i) = v + W/2*omega(i) + 1/2*correction;
    vl(i) = v - W/2*omega(i) - 1/2*correction;
    robot.sendVelocity(vl(i), vr(i));
    v_left = ds_left/dt;
    v_right = ds_right/dt;
    [X_R, Y_R, TH_R] = modelDiffSteerRobot(v_left, v_right, 0, T, dt);
    pause(0.05);
    %[X, Y, TH] = modelDiffSteerRobot(vl, vr, 0, T, dt);
    [X, Y, TH] = modelDiffSteerRobot(vl, vr, 0, T, DT);
    set(myPlot, 'xdata', X, 'ydata', Y);
    ptoc = T;
    i = i + 1;
end

%for i = 1:32
%    robot.sendVelocity(vl(i), vr(i));
%    pause(0.05);
%end

robot.stop();