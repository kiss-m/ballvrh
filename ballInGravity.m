clear all
close all
clc
Ts = 0.01;
Tmax = 10; %40
R = 0.9; % Coefficient of Restitution
 
%% Initialisation of the output vectors
tout = [];
Xout = [];
%% Initial Condition
X0 = [0;0.3;5;0];
t = 0;
 
%% Solver settings
options = odeset('Events', @bounceEvents, 'MaxStep',Ts);
%% Anonymous function
odeFcn = @EqBouncingBall;
while 1
 
    [t,X] = ode45(odeFcn,[t(end) Tmax],X0,options);
 
    tout = [tout;t];
    Xout = [Xout;X];
 
    X0 = X(end,:); %New initial condition
    X0(4) = -X0(4)*sqrt(R); % Velocity direction change
 
    %% Stop Condition
    if t(end) >= Tmax
        break;
    end
 
end
 
 
%% Extracting the position vector in x and y direction
y = Xout(:,3);
x = Xout(:,1);
 
%Plot
figure('units','normalized','outerposition',[0 0 1 1])
plot(x(1:10:end),y(1:10:end),'b.-','MarkerSize',10,'LineWidth',1.5)
hold on 
xlabel('$x [m]$', 'FontSize', 24,'interpreter','latex')
ylabel('$y [m]$', 'FontSize', 24,'interpreter','latex')
title('$Bouncing Ball$', 'FontSize', 24,'interpreter','latex')
grid on
xlim([0,x(end)])
function [dX] = EqBouncingBall(t,X)
 
g = 9.81;
m = 2;
 
A = [0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B =  [0; 0; 0; -g];
dX = A *X + B;
 
end
function [value,isterminal,direction] = bounceEvents(t,y)
    value = y(3);     % Detect height = 0
    isterminal = 1;   % Stop the integration
    direction = -1;   % Negative direction only
end