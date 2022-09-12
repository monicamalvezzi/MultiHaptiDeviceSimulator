function [out]=plotdevice(points,F,xe,ye)

% points
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

% 

axis('equal')
plot(xC,yC,'ro')
hold on
plot(xE,yE,'ro')
plot(0,0,'ko')
plot(xD,yD,'ro')
plot(xF,yF,'ro')
plot(xA,yA,'bo')
plot(xB,yB,'bo')
plot(xP,yP,'bo')

plot([xC xE],[yC yE],'r','LineWidth',2)
plot([xC xD],[yC yD],'b','LineWidth',2)
plot([xF xE],[yF yE],'b','LineWidth',2)
plot([xD xA],[yD yA],'b','LineWidth',2)
plot([xF xB],[yF yB],'b','LineWidth',2)
plot([xA xB],[yA yB],'b--','LineWidth',2)
plot([xA xP],[yA yP],'k','LineWidth',2)
plot([xP xB],[yP yB],'k','LineWidth',2)
%
% force
quiver(xP,yP,F(1),F(2),0.01,'LineWidth',2)

% kinematic ellipsoid
plot(xe,ye)

hold off

out = 1;