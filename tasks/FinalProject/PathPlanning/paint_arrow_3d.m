function head = paint_arrow_3d(arrow_front,arrow_tail,length)
arrow_direction=arrow_front-arrow_tail;
reference=[0;1];

angle_y = atan2(det([[arrow_direction(1);arrow_direction(3)],reference]),dot([arrow_direction(1);arrow_direction(3)],reference));
angle_x = atan2(det([[arrow_direction(2);arrow_direction(3)],reference]),dot([arrow_direction(2);arrow_direction(3)],reference));

if arrow_direction(3)>0
    head1=[0;0;-length];
else
    head1=[0;0;+length];
end
head2=head1;
head3=head1;
head4=head1;

angle_x=angle_x+pi/4;
R_y=[cos(angle_y),0,sin(angle_y);0,1,0;-sin(angle_y),0,cos(angle_y)];
R_x=[1,0,0;0,cos(angle_x),-sin(angle_x);0,sin(angle_x),cos(angle_x)];
R_z2=[cos(pi/2),-sin(pi/2),0;sin(pi/2),cos(pi/2),0;0,0,1];
R_z3=[cos(pi/2),sin(pi/2),0;-sin(pi/2),cos(pi/2),0;0,0,1];
R_z4=[cos(pi),sin(pi),0;-sin(pi),cos(pi),0;0,0,1];

head1=R_y*R_x*head1+arrow_front;
head2=R_y*R_z2*R_x*head2+arrow_front;
head3=R_y*R_z3*R_x*head3+arrow_front;
head4=R_y*R_z4*R_x*head4+arrow_front;

head=[head1,head2,head3,head4];
end