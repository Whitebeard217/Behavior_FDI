clear;
close all;
clc;
% c)
A = [0,1;0,0];
B =[0;1];
x0 = [2;10];
ad
[x, u, pf, t] = subOptimalSimulation(A,B,r,x0);
x1= x(1,:);x2 = x(2,:);
%%% State Trajectory Plot %%%
figure;
plot(t,x1)
hold on
plot(t,x2)
hold off
grid on
title('SubOptimal Solution State Trajectory Plot')
xlabel('Time, t')
ylabel('State Variables x1 and x2')
legend({'x1(t)','x2(t)'},'Location', 'northeast')
%%% Control Input Plot %%%
figure;
plot(t,u);
grid on
title('SubOptimal Solution State Control Input Plot ')
xlabel('Time, t')
ylabel('Control Input U(t)')
