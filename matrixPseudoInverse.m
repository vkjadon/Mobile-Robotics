%% Comments
% Moore-Penrose Pseudo Inverse can be 
% right inverse or left inverse. 
% It depends on the number of rows and columns
% of the rectangular matrix
%% Solution of Equations
% a11.x1+a12.x2=b1 a21.x1+a22.x2=b2 a31.x1+a32.x2=b3
a11=1;a12=-3;a21=3;a22=7;a31=1;a32=-7; b1=8;b2=2;b3=10;
A=[a11,a12;a21,a22;a31,a32]; b=[b1;b2;b3]
disp(A);
A_pi=pinv(A);
disp(A_pi*A); %check the Product of Pinv and A
x=A_pi*b
%disp(x(2))
disp(a11*x(1)+a12*x(2))
disp(a21*x(1)+a22*x(2))
disp(a31*x(1)+a32*x(2))
x1=(-10:10);
%% Plot
figure
plot(x1,(b1-a11*x1)/a12)
hold on
plot(x1,(b2-a21*x1)/a22)
hold on
plot(x1,(b3-a31*x1)/a32)