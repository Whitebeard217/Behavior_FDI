
% b) tf= 100sec
A = [0,1;0,0];
B =[0;1];
x0 = [5;10];
adb
tRange = [-100:0.1:0];
[x, u, pf, t] = optimalSystemSimulation(A,B,r,x0,tRange);
x1= x(1,:);
x2 = x(2,:);
%%% State Trajectory Plot %%%
figure;
plot(t,x1)
hold on
plot(t,x2)
hold off
grid on
title('State Trajectory Plot for t=100sec')
xlabel('Time, t')
ylabel('State Variables x1 and x2')
legend({'x1(t)','x2(t)'},'Location', 'northeast')
%%% Control Input Plot %%%
figure;
plot(t,u);
grid on
title('Control Input Plot for t=100sec')
xlabel('Time, t')
ylabel('Control Input U(t)')