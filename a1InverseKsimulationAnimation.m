%% Simulation of MR
clear ALL; clc; close all;
%newPosition=oldPosition+posDerivative*dt
%% Simulation Paraments
dt=0.1; ts=10; 
tsArray=0:dt:ts;
%disp(tsArray);
%% Initial Conditions
x0=3; y0=5; psi0=0;
eta0=[x0, y0, psi0];
eta(:,1)=eta0;
revolutions=1.5;
omega=revolutions*(2*pi)/ts;% 2pi radians in ts Seconds for revolutions
%disp(length(tsArray))
for i=1:length(tsArray)
    psi=eta(3,i);
    Jacobian=[cos(psi), -sin(psi), 0;
            sin(psi), cos(psi), 0;
            0, 0, 1];
    etaDesired=[2-2*cos(omega*tsArray(i));
                2*sin(omega*tsArray(i));
                omega*tsArray(i);];
    etaDotDesired=[2*omega*sin(omega*tsArray(i));
                2*omega*cos(omega*tsArray(i));
                omega;];
    zeta(:,i)=inv(Jacobian)*etaDotDesired;
    etaDot(:,i)=Jacobian*zeta(:,i);
    eta(:,i+1)=eta(:,i)+dt*etaDot(:,i);
    error(:,i)=etaDesired-eta(:,i);
end
%disp(eta(3,:));
%% Plot
roboLength=0.6;
roboWidth=0.4;

%% Initialize video
myVideo = VideoWriter('Mobile Robo Simulation');
myVideo.FrameRate = 10;
open(myVideo);

roboModel=[-roboLength/2,roboLength/2,roboLength/2,-roboLength/2,-roboLength/2;
            -roboWidth/2, -roboWidth/2, roboWidth/2, roboWidth/2,-roboWidth/2,];
for i=1:length(tsArray)
    psi=eta(3,i);
    ctm=[cos(psi), -sin(psi);
        sin(psi), cos(psi)];
    %Orienting Robo wrt Body Fixed Frame
    %Using Coordinate Transformation or Rotation Matrix R(z)
    robo=ctm*roboModel;
    %Tranforming robo wrt Inertial Frame
    fill(robo(1,:)+eta(1,i), robo(2,:)+eta(2,i), 'g');
    %plot(robo(1,:),robo(2,:));
    axis([-1 10 -1 10]);
    hold on
    plot(eta(1,1:i),eta(2,1:i));
    legend('Robo', 'Trajectory')
    hold on;
    pause(0.1);
    hold off;
    
    frame = getframe(gca); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)