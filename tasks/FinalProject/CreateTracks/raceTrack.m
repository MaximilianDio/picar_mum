clc
clear all
close all
%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%
racetrack = [];

line1 = getLine([1;0;0],[4;0;0]);
line2 = getLine([4;2;0],[1;2;0]);

curve1= getCircle(1,[270;450],[4;1;0]);
curve2= getCircle(0.75,[90;180],[4;1.25;0]);
curve3= flipud(getCircle(0.75,[180;360],[2.5;1.25;0]));
curve4= getCircle(0.75,[0;90],[1;1.25;0]);
curve5= getCircle(1,[90;270],[1;1;0]);
%% create track
racetrack = [line1;curve1;curve2;curve3;curve4;curve5];
testtrack = [line1;curve1;line2;curve5];

%% plot track
figure
plot(racetrack(:,1),racetrack(:,2))
axis equal
title('racetrack')

figure
plot(testtrack(:,1),testtrack(:,2))
axis equal
title('testtrack')



%% save tracks in txt files
writematrix(reshape(racetrack',[1,prod(size(racetrack))]),'race_track.txt','Delimiter','space')
writematrix(reshape(testtrack',[1,prod(size(testtrack))]),'test_track.txt','Delimiter','space')