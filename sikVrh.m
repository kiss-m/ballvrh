clear all
close all
clc
 
%% Parameters
tSpan = 7.2;
m = 2;
h = 10e-2;
v0 = 50;
phi = pi/4;
ic = [0; cos(phi)*v0; 0 ;sin(phi)*v0];
%% Runge-Kutta Method
n = tSpan/h;
tRK = 0:h:tSpan;
xRK = ic;
for i = 2:n+1
 
    a1 = odeThrowingAtAnAngle(tRK(i-1),xRK(:,i-1));
    a2 = odeThrowingAtAnAngle(tRK(i-1),xRK(:,i-1) + h/2*a1);
    a3 = odeThrowingAtAnAngle(tRK(i-1),xRK(:,i-1) + h/2*a2);
    a4 = odeThrowingAtAnAngle(tRK(i-1),xRK(:,i-1) + h*a3);
 
 
    xRK(:,i) = xRK(:,i-1) + h/6*(a1 + 2*a2 + 2*a3 + a4);
 
end
 
%% ODE45
[t,xODE45] = ode45(@odeThrowingAtAnAngle,[0,tSpan],ic);
 
%% Plot
subplot(2,1,1)
plot(xODE45(:,1),xODE45(:,3))
grid on
hold on
plot(xRK(1,:),xRK(3,:))
legend('ODE45','Runge-Kutta')
xlabel('t [s]')
ylabel('x_1')
 
subplot(2,1,2)
plot(xODE45(:,2),xODE45(:,4))
grid on
hold on
plot(xRK(2,:),xRK(4,:))
legend('ODE45','Runge-Kutta')
xlabel('x_4')
ylabel('x_2')
 
 
%% Total Kinetic Energy
Ek = 1/2*m*(xODE45(:,2).^2 + xODE45(:,4).^2);
 
figure
plot(t,Ek)
xlabel('t[s]')
ylabel('Ek [J]')
grid on 
%% Total Kinetic Energy
totalKineticEnergy = sum(Ek);
fprintf('Total Kinetic Energy: %3.2f \n',totalKineticEnergy)
 
%% Total Distance Travelled
totalDistanceTravelled = sum(sqrt(diff(xODE45(:,1)).^2 + diff(xODE45(:,3)).^2));
fprintf('Total Distance Travelled: %3.2f \n',totalDistanceTravelled)
 
%% Initial condition for vy
% y = v0*sin(phi)*t - 1/2*g*t^2
% v0_est = 1/2*g*t/sin(phi)
 
v0_est = 1/2*9.81*t/sin(phi);
 
fprintf('Initial velocity: %3.2f \n',v0_est)
function [dX] = odeThrowingAtAnAngle(t,x)
 
g = 9.81;
 
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; 0; 0; -g];
dX = A*x + B;
 
end