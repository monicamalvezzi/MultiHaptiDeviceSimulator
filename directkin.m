function [points, var, aus]=directkin(q,geom)
l0 = geom(1);
l1 = geom(2);
l2 = geom(3);
l3 = geom(4);
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% points
xC = -l0/2;
yC = 0;

xE = l0/2;
yE = 0;

xD = xC+l1*cos(q1);
yD = yC+l1*sin(q1);

xF = xE+l1*cos(q3);
yF = yE+l1*sin(q3);

xA = xD+l2*cos(q1+q2);
yA = yD+l2*sin(q1+q2);

xB = xF+l2*cos(q3+q4);
yB = yF+l2*sin(q3+q4);

L = sqrt((xB-xA)^2+(yB-yA)^2);
theta = atan2((yB-yA),(xB-xA));
alpha = 2*asin(L/(2*l3));

xP = xA+l3*cos(theta+(pi-alpha)/2);
yP = yA+l3*sin(theta+(pi-alpha)/2);

points=[xC, yC, xE, yE, xD, yD, xF, yF, xA, yA, xB, yB, xP, yP];
var=[xP yP theta alpha];
aus = L;
