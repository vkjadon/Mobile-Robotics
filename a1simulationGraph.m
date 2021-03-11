%% Simulation of MR
clear ALL; clc; close all;
%x+=x+x'dt
%% Simulation Paraments
dt=0.1; ts=10; 
tsArray=0:dt:ts;
%disp(tsArray);
%% Initial Conditions
x0=0; y0=0; psi0=0; psi=0;
u=10.1; v=0; r=0;
eta0=[x0, y0, psi0];
eta(:,1)=eta0;
zeta=[u; v; r];
%disp(eta);
for i=1:length(tsArray)
    Jacobian=[cos(psi), -sin(psi), 0;
            sin(psi), cos(psi), 0;
            0, 0, psi];
    etaDot(:,i)=Jacobian*zeta;
    eta(:,i+1)=eta(:,i)+dt*etaDot(:,i);
    psi=eta(3,i+1);
    disp(eta);
end
%%Plot
plot(tsArray, eta(1,1:i));