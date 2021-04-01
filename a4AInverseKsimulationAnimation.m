%% Simulation of MR
clear ALL; clc; close all;
%newPosition=oldPosition+posDerivative*dt
%% Simulation Paraments
dt=0.1; ts=4; 
tsArray=0:dt:ts;
%disp(tsArray);
%% Initial Conditions
revolutions=1.0;
omega=revolutions*(2*pi)/ts;% 2pi radians in ts Seconds for revolutions
%Center and Radius for Circular Trajectory wrt Inertial Frame
xc=-10;yc=-2; rho=3;
%Initial Position of Robot wrt Inertial Frame
psi0=0; x0=xc+rho*cos(psi0); y0=yc+rho*sin(psi0);
eta0=[x0, y0, psi0];
eta(:,1)=eta0;
%disp(length(tsArray))
for i=1:length(tsArray)
    psi=eta(3,i);
    Jacobian=[cos(psi), -sin(psi), 0;
            sin(psi), cos(psi), 0;
            0, 0, 1];
    etaDesired=[xc+rho*cos(psi0+omega*tsArray(i));
                yc+rho*sin(psi0+omega*tsArray(i));
                psi0+omega*tsArray(i);];
    etaDotDesired=[-rho*omega*sin(psi0+omega*tsArray(i));
                rho*omega*cos(psi0+omega*tsArray(i));
                omega;];
    zeta(:,i)=inv(Jacobian)*etaDotDesired;
    etaDot(:,i)=Jacobian*zeta(:,i);
    eta(:,i+1)=eta(:,i)+dt*etaDot(:,i);
    error(:,i)=etaDesired-eta(:,i);
    %str=sprintf('psi %f Des %f', eta(3,i)*(180*7/22), etaDesired(3)*(180*7/22))
    str=sprintf('Error %f ',error(3,i) )
end
%disp(error(3,:));
%% Plot
for i=1:length(tsArray)
    %axis([-10 10 -10 10]);
    plot(eta(1,1:i),eta(2,1:i));
    legend('Trajectory')
    hold on;
    pause(0.1);
    hold off;
end