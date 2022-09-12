function [J,J1]=devicejacobian1(points)


xC = points(1);
yC = points(2);
xE = points(3);
yE = points(4);

xD = points(5);
yD = points(6);

xF = points(7);
yF = points(8);

xA = points(9);
yA = points(10);

xB = points(11);
yB = points(12);

xP = points(13);
yP = points(14);

A = [-(yA-yC) -(yA-yD) (yB-yE) (yB-yF);
    (xA-xC) (xA-xD) -(xB-xE) -(xB-xF)];

B = [(yP-yA) -(yP-yB);
    -(xP-xA) (xP-xB)];

C = [-(yA-yC) -(yA-yD) 0 0;
    (xA-xC) (xA-xD) 0 0];

D = [-(yP-yA);
    xP-xA];

J = [C+D*[1 0]*inv(B)*A;
    [1 1]*inv(B)*A;
    [1 -1]*inv(B)*A];


C1 = [-(yB-yE) -(yB-yF) 0 0;
    (xB-xE) (xB-xF) 0 0];

D1 = [-(yP-yB);
    xP-xB];

J1 = [C1+D1*[0 -1]*inv(B)*A;
    [1 1]*inv(B)*A;
    [1 -1]*inv(B)*A];


