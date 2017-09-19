close all;
clear all;
clc;

robot = raspbot();
robot.encoders.NewMessageFcn=@encoderEventListener;
robot.sendVelocity(0, 0);
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
global v_left;
global v_right;
global dt;
global encI;
global encDT;

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
VL = zeros(1);
VR = zeros(1);

DT = zeros(1);
error = 0.003;


encI = 1;
v_left = zeros(1);
v_right = zeros(1);
encDT = zeros(1);
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
        correction = error;
    else
        correction = -error;
    end
    vr(i) = v + W/2*omega(i) + 1/2*correction;
    vl(i) = v - W/2*omega(i) - 1/2*correction;
    robot.sendVelocity(vl(i), vr(i));
    %VL(i) = v_left;
    %VR(i) = v_right;
    %[X_R, Y_R, TH_R] = modelDiffSteerRobot(v_left, v_right, 0, T, dt);
    pause(0.05);
    %[X, Y, TH] = modelDiffSteerRobot(vl, vr, 0, T, dt);
    [X, Y, TH] = modelDiffSteerRobot(vl, vr, 0, T, DT);
    %set(myPlot, 'xdata', X_R, 'ydata', Y_R);
    ptoc = T;
    i = i + 1;
end

%for i = 1:32
%    robot.sendVelocity(vl(i), vr(i));
%    pause(0.05);
%end

robot.encoders.NewMessageFcn=[];
robot.stop();