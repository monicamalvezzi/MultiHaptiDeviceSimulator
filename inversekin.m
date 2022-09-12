function [points, q, aus]=inversekin(nu,geom)
l0 = geom(1);
l1 = geom(2);
l2 = geom(3);
l3 = geom(4);
xP=nu(1);
yP=nu(2);
theta=nu(3);
alpha=nu(4);

L = 2*l3*sin(alpha/2);
qA = theta +(pi-alpha)/2;
qB = qA+alpha;

xC = -l0/2;
yC = 0;

xE = l0/2;
yE = 0;

xA = xP - l3*cos(qA);
yA = yP - l3*sin(qA);

xB = xP - l3*cos(qB);
yB = yP - l3*sin(qB);

lAC=sqrt((xA-xC)^2+(yA-yC)^2);
lBE=sqrt((xB-xE)^2+(yB-yE)^2);

qAC = atan2((yA-yC),(xA-xC));
qBE = atan2((yB-yE),(xB-xE));

deltaC = acos((l1^2+lAC^2-l2^2)/(2*l1*lAC));
q1 = qAC-deltaC;
deltaD = acos((l1^2+l2^2-lAC^2)/(2*l1*l2));
q2=pi-deltaD;

deltaE = acos((l1^2+lBE^2-l2^2)/(2*l1*lBE));
q3 = (qBE+deltaE);

deltaF = acos((l1^2+l2^2-lBE^2)/(2*l1*l2));
q4=-deltaF;

xD = xC+l1*cos(q1);
yD = yC+l1*sin(q1);

xF = xE+l1*cos(q3);
yF = yE+l1*sin(q3);



points=[xC, yC, xE, yE, xD, yD, xF, yF, xA, yA, xB, yB, xP, yP];
q=[q1 q2 q3 q4];
aus = L;
