% -------------------------------------------------------------------------
% Autor    : Matti
% Umgebung : Matlab R2015B
% Stand    : 07.02.2017
% -------------------------------------------------------------------------
% Visualisierung des Autos in 2D
close all;
clear all;
clc;

%% User Presets
adjust_pause=0.0;%sim rate? 0.1:slower | 0.01: bit faster | 0.0:MAX-SPEED
disp_path=true;%show path? true | false (boolean)
window_scale=0.5;%1:full screen=bad performance | 0.5:half the size=good performance
capture_video = false; % save video? true/false
save_as_png=0;%save animation as png pictures?  1:true | 0:false 
velocity=0; %show velocity arrow? 1:true | 0:false
angle=0; %Show angle? 1:true | 0:false
vel_gamma=1; %show plot of velocity and stearing angle? 1:true | 0:false --> bad performance

%% read out data
% needed inputs:
% L:     scalar: wheelbase (Radstand)
% t:     vector: time (needed for the time display)
% theta: vector: orientation of car
% x:     vector: x position
% y:     vector: y position
% u:     vector: [v gamma] ([geschwindigkeit lenkung])
load('animation.mat')
L = A.L;
t_ = A.t;
theta = A.theta.';
u = A.u.';
x = A.x.';
y = A.y.';
RATE = 100;
[t__,x] = even_sample(t_.',x, RATE);
[t, u] = even_sample(t_.',u, RATE);
[t, theta] = even_sample(t_.',theta, RATE);
[t, y] = even_sample(t_.',y, RATE);

%% Car Dimensions
carLength=0.8 * L;
carWidth=carLength/2;
wheelWidth=carLength/4;
wheelLength=carLength/2;
arrowscale=0.1;

%% Object: Car
% the car is defined as 2 rectangles which represent the 2 wheels
% so we need 16 points:
% 8 points for the x position of the wheels
% 8 points for the y position of the wheels

%%
%rear wheel (XXX check sign XXX)
x_rw1=x-cos(-theta)*wheelLength/2+sin(-theta)*wheelWidth/2;
x_rw2=x+cos(-theta)*wheelLength/2+sin(-theta)*wheelWidth/2;
x_rw3=x+cos(-theta)*wheelLength/2-sin(-theta)*wheelWidth/2;
x_rw4=x-cos(-theta)*wheelLength/2-sin(-theta)*wheelWidth/2;
y_rw1=y+sin(-theta)*wheelLength/2+cos(-theta)*wheelWidth/2;
y_rw2=y-sin(-theta)*wheelLength/2+cos(-theta)*wheelWidth/2;
y_rw3=y-sin(-theta)*wheelLength/2-cos(-theta)*wheelWidth/2;
y_rw4=y+sin(-theta)*wheelLength/2-cos(-theta)*wheelWidth/2;

rearWheel_x=[x_rw1,x_rw2,x_rw3,x_rw4];
rearWheel_y=[y_rw1,y_rw2,y_rw3,y_rw4];

%front wheel
%if gamma is a multiple of pi --> reduce gamma
gamma=u(:,2);
for i=1:length(gamma)
    while gamma(i)>2*pi
        gamma(i)=gamma(i)-2*pi;
    end
end

x_fw1=x+carLength*cos(theta)-cos(-theta-gamma)*wheelLength/2+sin(-theta-gamma)*wheelWidth/2;
x_fw2=x+carLength*cos(theta)+cos(-theta-gamma)*wheelLength/2+sin(-theta-gamma)*wheelWidth/2;
x_fw3=x+carLength*cos(theta)+cos(-theta-gamma)*wheelLength/2-sin(-theta-gamma)*wheelWidth/2;
x_fw4=x+carLength*cos(theta)-cos(-theta-gamma)*wheelLength/2-sin(-theta-gamma)*wheelWidth/2;
y_fw1=y+carLength*sin(theta)+sin(-theta-gamma)*wheelLength/2+cos(-theta-gamma)*wheelWidth/2;
y_fw2=y+carLength*sin(theta)-sin(-theta-gamma)*wheelLength/2+cos(-theta-gamma)*wheelWidth/2;
y_fw3=y+carLength*sin(theta)-sin(-theta-gamma)*wheelLength/2-cos(-theta-gamma)*wheelWidth/2;
y_fw4=y+carLength*sin(theta)+sin(-theta-gamma)*wheelLength/2-cos(-theta-gamma)*wheelWidth/2;

frontWheel_x=[x_fw1,x_fw2,x_fw3,x_fw4];
frontWheel_y=[y_fw1,y_fw2,y_fw3,y_fw4];

%connectionline of the two tires
x1=x;
x2=x+carLength*cos(theta);
y1=y;
y2=y+carLength*sin(theta);

Line_x=[x1,x2];
Line_y=[y1,y2];

%record animation
if capture_video
    figure_title='RECORDING';
    video = VideoWriter('Animation.avi');
    video.FrameRate=round(1/(t(2)-t(1)));
    open(video);
else
    figure_title='NOT RECORDING';
end

%%
%Compute velocity of the rear wheel numerically
h=t(2)-t(1);
%Velocity
dx=diff(x)/h; %differentiate numerically
dy=diff(y)/h;

v=[dx,dy]; %velocity of the rear wheel
pos=[x,y]; %position of the rear wheel
v(length(t),:)=v(length(t)-1,:); %due to the diff command the length of the vector got reduced by one --> use last value
arrow_pos=pos+arrowscale.*v; %end of the velocity vector

arrow_heads=zeros(2,5,length(t));

for ii=1:length(t+1)
    arrow_heads(:,:,ii)=paint_arrow_2d(pos(ii,:).',arrow_pos(ii,:).',carLength/5);
end
%%
%Angle
S1=zeros(2,2,length(t)); %rotationmatrix (theta+gamma)
S2=zeros(2,2,length(t)); %rotationmatrix theta
angleLine1=zeros(2,length(t));  
angleLine2=zeros(2,length(t));
for jj=1:length(t)
    S1(:,:,jj)=[cos(-theta(jj)-gamma(jj)),sin(-theta(jj)-gamma(jj));-sin(-theta(jj)-gamma(jj)),cos(-theta(jj)-gamma(jj))];
    S2(:,:,jj)=[cos(-theta(jj)),sin(-theta(jj));-sin(-theta(jj)),cos(-theta(jj))];
    angleLine1(:,jj)=[x2(jj);y2(jj)]+S1(:,:,jj)*[1.5*carLength;0];
    angleLine2(:,jj)=[x2(jj);y2(jj)]+S2(:,:,jj)*[carLength*1.5;0];
end



%% Plot
% window
window(1)=min(x)-carLength;%x min
window(2)=max(x)+carLength;%x max
window(3)=min(y)-carLength;%y min
window(4)=max(y)+carLength;%y max

Pix_Screen=get(0,'screensize');%get dimension of PC screen

figure('Name','Car-Animation','NumberTitle','off','position',...
    [(Pix_Screen(3)-Pix_Screen(3)*window_scale)/2,...%shift
    (Pix_Screen(4)-Pix_Screen(4)*window_scale)/2,...%shift
    Pix_Screen(3)*window_scale,...%window size
    Pix_Screen(4)*window_scale])%window size
% xlim = ([-1 50]);
% ylim = ([-1 3]);
% set(gca,'DataAspectRatio',[1 1 1])%no distorted dimension
hold on


if vel_gamma==1
    subplot('position',[0.1,...%shift
    0.1,...%shift
    0.5,...%window size
    0.8])
end
axis(window)
%%
%Dfinitions
rearWheel=patch(rearWheel_x(1,:),rearWheel_y(1,:),[0.7 0.7 0.7]);
frontWheel=patch(frontWheel_x(1,:),frontWheel_y(1,:),[0.7 0.7 0.7]);
connectionLine=line(Line_x(1,:),Line_y(1,:),'LineWidth',1.7,'Color',[0.2 0.2 0.2]);
% hinterreifen.FaceColor=[0.5 0.5 0.5];
if disp_path == true%plot path: true / false
    if vel_gamma==1
        subplot('position',[0.1,...%shift
        0.1,...%shift
        0.5,...%window size
        0.8])
    end
    hold on
    path=plot(x(1),y(1),':r');
end

if velocity==1
    if vel_gamma==1
        subplot('position',[0.1,...%shift
        0.1,...%shift
        0.5,...%window size
        0.8])
    end
    vel_arrow=plot([arrow_pos(1,1),pos(1,1)],[arrow_pos(1,2),pos(1,2)],'Color',[0,0.5,1],'LineWidth',1.5);
    vel_arrow1=plot([arrow_pos(1,1),arrow_heads(1,3,1)],[arrow_pos(1,2),arrow_heads(2,3,1)],'Color',[0,0.5,1],'LineWidth',1);
    vel_arrow2=plot([arrow_pos(1,1),arrow_heads(1,5,1)],[arrow_pos(1,2),arrow_heads(2,5,1)],'Color',[0,0.5,1],'LineWidth',1.5);
end
if angle==1
    if vel_gamma==1
        subplot('position',[0.1,...%shift
        0.1,...%shift
        0.5,...%window size
        0.8])
    end
    angle_line1=plot([x2(1),angleLine1(1,1)],[y2(1),angleLine1(2,1)],'m','LineWidth',1);
    angle_line2=plot([x2(1),angleLine2(1,1)],[y2(1),angleLine2(2,1)],'m','LineWidth',1);
    x_bow=zeros(20,1);
    y_bow=zeros(20,1);
    steps=linspace(0,gamma(1),20);
    for k=1:20 %Create bow
            x_bow(k)=x2(1)+cos(theta(1)+steps(k))*1.3*carLength;
            y_bow(k)=y2(1)+sin(theta(1)+steps(k))*1.3*carLength;
    end
    if vel_gamma==1
        subplot('position',[0.1,...%shift
           0.1,...%shift
        0.5,...%window size
        0.8])
    end
    bow=plot(x_bow,y_bow,'m','LineWidth',1);
end

if vel_gamma==1
    subplot('position',[0.675,0.65,0.3,0.25])
    curve1=animatedline('Color','b');
    grid on
    title('Velocity')
    set(gca,'XLIM',[0 t(end)],'YLIM',[1.1*min(min(v)) 1.1*max(max(v))]) 
    xlabel('time in [s]')
    ylabel('v in [m/s]')

    subplot('position',[0.675,0.2,0.3,0.25])
    curve2=animatedline('Color','m');
    grid on
    title('Stearing Angle')
    set(gca,'XLIM',[0 t(end)],'YLIM',[1.1*min(gamma) 1.1*max(gamma)])
    xlabel('time in [s]')
    ylabel('\gamma in [rad]')
end

%%
%png
if save_as_png==1
    print(strcat('AnimationVideo\Animation_',num2str(1)),'-dpng');%save as png picture
end
%% Capture 1st frame
if capture_video
    frame=getframe(gcf);%initial frame of the video
    writeVideo(video,frame);
end
tic
for i = 2:length(t)
    if vel_gamma==1
    subplot('position',[0.1,...%shift
    0.1,...%shift
    0.5,...%window size
    0.8])
    end
    title(['time t = ', num2str(t(i), '%0.2f s')])
    set(rearWheel,'XData',rearWheel_x(i,:),'YData',rearWheel_y(i,:))
    set(frontWheel,'XData',frontWheel_x(i,:),'YData',frontWheel_y(i,:))
    set(connectionLine,'XData',Line_x(i,:),'YData',Line_y(i,:))
    if disp_path == true%plot path: true / false
        set(path,'XData',x(1:i),'YData',y(1:i))%enhace path
    end
    drawnow nocallbacks
    pause(adjust_pause)%pause (define by the user)
    if capture_video
        frame=getframe(gcf);
        writeVideo(video,frame);
    end
    
    if velocity==1 && i<length(t)-1
        set(vel_arrow,'XData',[arrow_pos(i,1),pos(i,1)],'YData',[arrow_pos(i,2),pos(i,2)]);
        set(vel_arrow1,'XData',[arrow_pos(i,1),arrow_heads(1,3,i)],'YData',[arrow_pos(i,2),arrow_heads(2,3,i)]);
        set(vel_arrow2,'XData',[arrow_pos(i,1),arrow_heads(1,5,i)],'YData',[arrow_pos(i,2),arrow_heads(2,5,i)]);
    end
    if angle==1
        set(angle_line1,'XDATA',[x2(i),angleLine1(1,i)],'YDATA',[y2(i),angleLine1(2,i)]);
        set(angle_line2,'XDATA',[x2(i),angleLine2(1,i)],'YDATA',[y2(i),angleLine2(2,i)]);
        steps=linspace(0,gamma(i),20);
        for k=1:20
            x_bow(k)=x2(i)+cos(theta(i)+steps(k))*1.3*carLength;
            y_bow(k)=y2(i)+sin(theta(i)+steps(k))*1.3*carLength;
        end
        set(bow,'XDATA',x_bow,'YDATA',y_bow);
    end
    
    if save_as_png==1
        print(strcat('AnimationVideo\Animation_',num2str(i)),'-dpng');%save as png picture
    end
    
    if vel_gamma==1
        subplot('position',[0.675,0.65,0.3,0.25])
        addpoints(curve1,t(i),norm(v(i,:)));
        drawnow nocallbacks
    
        subplot('position',[0.675,0.2,0.3,0.25])
        addpoints(curve2,t(i),gamma(i));
        drawnow nocallbacks
    end
end
time=toc;
if capture_video
    close(video);%close video file
end

fprintf('Plotting time: %d seconds\n',time)