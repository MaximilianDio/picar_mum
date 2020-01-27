function[points]= getCircle(radius,angles,center)
%       [points]= getCircle(radius,angles,center)
%       
%       calculates part of a circle and returns points of a circle
%
%   input
%       radius      ......      [scalar]        radius of the circle
%       angles      ......      [2x1 vector]    start and end angle
%       (mathematical positiv rotation)
%       center      ......      [3x1 vector]    center of the circle
%
%   output
%       points      ......      [n x 3 matricx] 3 d points of the circle


%% init 
pi= 3.1416;

Dalpha = 0.02/radius;
alpha = transpose((angles(1)/180*pi):Dalpha:(angles(2)/180*pi));


points = center' + [cos(alpha),sin(alpha),zeros(length(alpha),1)]* radius;

