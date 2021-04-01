%% Simulation of MR
clear ALL; clc; close all;
%newPosition=oldPosition+posDerivative*dt
%% Simulation Paraments
dt=1; ts=50; 
tsArray=0:dt:ts;
%disp(tsArray);
%% Initial Conditions
x0=3; y0=5; psi0=0;
eta0=[x0, y0, psi0];
eta(:,1)=eta0;
%% MR Parameter in centimeters
a=1.0; d=2.5;
W=[a/2, a/2;
   0,0;
   -a/(2*d), a/(2*d)];
%% Simulation
omega_1=0.0; omega_2=0.5; 
omega=[omega_1;omega_2];    
%disp(length(tsArray))
for i=1:length(tsArray)
    psi=eta(3,i);
    Jacobian=[cos(psi), -sin(psi), 0;
            sin(psi), cos(psi), 0;
            0, 0, 1];
    zeta(:,i)=W*omega;
    etaDot(:,i)=Jacobian*zeta(:,i);
    eta(:,i+1)=eta(:,i)+dt*etaDot(:,i);
end
disp(eta);
%% Plot
roboLength=0.6;
roboWidth=0.4;
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
    pause(0.01);
    hold off;
end