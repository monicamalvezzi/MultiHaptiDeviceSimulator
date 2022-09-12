close all
clear all
clc

% geom parameters
l0 = 0.0813;
l1 = 0.09052;
l2 = 0.035;
l3 = 0.03202;

% angles
q10=-pi/2;
q30=-pi/2;
q20=pi/2;
q40=-pi/2;


q1m = -10*pi/180;
q2m = 0;
q3m = 10*pi/180;
q4m = 0;

q1 = q10+q1m; 
q2 = q20+q2m; 
q3 = q30+q3m; 
q4 = q40+q4m; 

q = [q1 q2 q3 q4];
geom = [l0 l1 l2 l3];
% forward-direct kinematics
[points,var,aus]=directkin(q,geom);
F= [0 -2 0 0]';

figure(1)

out = plotdevice(points,F,0,0);

% differential kinematics, evaluation of device Jacobian matrix

J=devicejacobian(q,geom);

tau = J'*F;
% inverse kinematics- test on trajectories defined in the end-effector
% space
xP = points(13);
yP = points(14);

    [J,J1]=devicejacobian1(points);
    JP= J(1:2,:);
    tau = J'*F;
    
    % manipulability ellipsoid
    [V,D]= eig((JP*JP'));
    
    s1 = sqrt(D(1,1));
    s2 = sqrt(D(2,2));
    
    t=[0:0.01:2*pi];
    scala = 0.2;
    xeo =scala*s1*cos(t);
    yeo =scala*s2*sin(t);
    
    for k = 1:length(t)
        vr = V*[xeo(k);yeo(k)];
        xe(k) =xP +vr(1);
        ye(k) =yP +vr(2);
    end      
    
    %tautraject(i,:)=tau';
    out = plotdevice(points,F,xe,ye);
% trajectories
%test 1 - horizontal
% time = [0:0.1:10];
% xPv = 0.05*time/max(time)-0.025;
% yPv = points(14)*ones(size(time));
% thetav = 0.0*time;
% alphav = pi/2*ones(size(time));

% test 2 - vertical
time = [0:0.1:10];
xPv = 0.0*time;
yPv = points(14)+0.025*time/max(time);
thetav = 0.0*time;
alphav = pi/2*ones(size(time));
% % 
% 
% % % test 3 - rotate
%  time = [0:0.1:10];
%  xPv = 0.0*time;
%  yPv = points(14)*ones(size(time));
%  thetav = time/max(time)-0.5;
%  alphav = pi/2*ones(size(time));
% 
figure(2)
axis('equal')
axis([-0.09 0.09 -0.12 0.01])

for i=1:length(time)
    xP = xPv(i);
    yP = yPv(i);
    theta = thetav(i);
    alpha = alphav(i);
    
    nu = [xP yP  theta alpha];
    
    [pointsinv, qcalc, aus]=inversekin(nu,geom);
    [J,J1]=devicejacobian1(pointsinv);
    JP= J(1:2,:);
    tau = J'*F;
    
    % manipulability ellipsoid
    [V,D]= eig((JP*JP'));
    
    s1 = sqrt(D(1,1));
    s2 = sqrt(D(2,2));
    
    t=[0:0.01:2*pi];
    scala = 0.2;
    xeo =scala*s1*cos(t);
    yeo =scala*s2*sin(t);
    
    for k = 1:length(t)
        vr = V*[xeo(k);yeo(k)];
        xe(k) =xP +vr(1);
        ye(k) =yP +vr(2);
    end      
    
    tautraject(i,:)=tau';
    out = plotdevice(pointsinv,F,xe,ye);
    axis([-0.09 0.09 -0.15 0.01])
    qtraject(i,:)=qcalc;
    pause(0.01);
end

figure(3)
subplot(2,2,1)
grid on
plot(time, qtraject(:,1)*180/pi,'LineWidth',2);
ylabel('Angle (deg)')
legend('Joint C')
subplot(2,2,2)
grid on
plot(time, qtraject(:,2)*180/pi,'LineWidth',2);
%ylabel('Angle (deg)')
legend('Joint D')
subplot(2,2,3)
grid on
plot(time, qtraject(:,3)*180/pi,'LineWidth',2);
ylabel('Angle (deg)')
legend('Joint E')
xlabel('Time (s)')
subplot(2,2,4)
grid on
plot(time, qtraject(:,4)*180/pi,'LineWidth',2);
%ylabel('Angle (deg)')
legend('Joint F')
xlabel('Time (s)')

figure(4)
subplot(2,2,1)
grid on
plot(time, tautraject(:,1),'LineWidth',2);
ylabel('Torque (Nm)')
legend('Joint C')
subplot(2,2,2)
grid on
plot(time, tautraject(:,2),'LineWidth',2);
%ylabel('Torque (Nm)')
legend('Joint D')
subplot(2,2,3)
grid on
plot(time, tautraject(:,3),'LineWidth',2);
ylabel('Torque (Nm)')
xlabel('Time (s)')
legend('Joint E')
subplot(2,2,4)
grid on
plot(time, tautraject(:,4),'LineWidth',2);
%ylabel('Torque (Nm)')
legend('Joint F')
xlabel('Time (s)')



