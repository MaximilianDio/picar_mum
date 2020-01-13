function x = paint_arrow_2d(p0,pf,length_of_head)
angle=atan2(pf(2)-p0(2),pf(1)-p0(1));
angle1=angle+45/180*pi;
angle2=angle-45/180*pi;
S1=[cos(angle1),-sin(angle1);sin(angle1),cos(angle1)];
S2=[cos(angle2),-sin(angle2);sin(angle2),cos(angle2)];

p1=pf-S1*[length_of_head;0];
p2=pf-S2*[length_of_head;0];

x=[pf,p0,p1,p0,p2];

% y=x(2,:);
% x=x(1,:);
end