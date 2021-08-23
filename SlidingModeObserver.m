% Implementation of "Second-Order Sliding-Mode Observer for Mechanical Systems" by Jorge Davila, LeonidFridman, andArie Levant
% Code Owner: Donghyeon Kim, kdh0429@snu.ac.kr

clear all; close all; clc


f = @systemFunction;
f_err = @errorDynamics;

t = [0:0.01:20];

%% System and Observer
x1_init = 0;
x2_init = 1;
x1_hat_init = 0;
x2_hat_init = 0;

[ts,ys] = ode45(f,t,[x1_init;x2_init;x1_hat_init;x2_hat_init]);
plot(ys(:,1),ys(:,2),'r') % Real System
hold on
plot(ys(:,3),ys(:,4),'b') % Observer


%% System Phase Portrait
x1 = linspace(-4,4,10);
x2 = linspace(-4,4,10);

[x,y] = meshgrid(x1,x2);

u = zeros(size(x));
v = zeros(size(x));

t=0; 
for i = 1:numel(x)
    xdot = f(t,[x(i); y(i); 0.0; 0.0]);
    u(i) = xdot(1);
    v(i) = xdot(2);
end

figure;
quiver(x,y,u,v,'r'); figure(gcf)
xlabel('x_1')
ylabel('x_2')
axis tight equal;

%% Error Dynamics Phase Portrait
x1_err = linspace(-10,10,10);
x2_err = linspace(-10,10,10);

[x_err,y_err] = meshgrid(x1_err,x2_err);

u_err = zeros(size(x_err));
v_err = zeros(size(x_err));

t=0; 
for i = 1:numel(x_err)
    xdot_err = f_err(t,[x_err(i); y_err(i)]);
    u_err(i) = xdot_err(1);
    v_err(i) = xdot_err(2);
end

figure;
quiver(x_err,y_err,u_err,v_err,'r'); figure(gcf)
xlabel('e_1')
ylabel('e_2')
axis tight equal;

%%
function y = systemFunction(t, x)
    M = 1.1; g=9.815; L=0.9; J=M*L^2; Vs= 0.18; Ps=0.45;
    x_d = sin(t);
    xdot_d = cos(t);
%     tau = -30*sign(x(1)-x_d) -15*(sign(x(2)-xdot_d));
    tau = -30*(x(1)-x_d) -15*(x(2)-xdot_d);
    v = 0.5*sin(2*t)+0.5*cos(2*t);
    
    % System
    y = zeros(4,1);
    y(1) = x(2);
    y(2) = 1/J*tau - g/L*sin(x(1)) - Vs/J*x(2) - Ps/J*sign(x(2)) + v;
    
    % Sliding Mode Observer
    Mn=1; Ln=1; Jn=Mn*Ln^2; Vsn=0.2; Psn=0.5;
    f=6;
    y(3) = x(4) + 1.5*sqrt(f)*sqrt(abs(x(1)-x(3)))*sign(x(1)-x(3));
    y(4) = 1/Jn*tau - g/Ln*sin(x(1)) - Vsn/Jn*x(4) + 1.1*f*sign(x(1)-x(3));
end

function y = errorDynamics(t,x)
    M = 1.1; g=9.815; L=0.9; J=M*L^2; Vs= 0.18; Ps=0.45;
    Mn=1; Ln=1; Jn=Mn*Ln^2; Vsn=0.2; Psn=0.5;
    f=6;
    
    y=zeros(2,1);
    % Super Twisting Algorithm
    y(1) = x(2)-1.5*sqrt(f)*sqrt(abs(x(1)))*sign(x(1));
    y(2) = -1.1*f*sign(x(1));
    % Simple Sliding Mode Observer
    %     y(1) = -1.0*x(1)+x(2)-2.0*sign(x(1));
    %     y(2) = -1.0*x(1)-2.0*sign(x(1));
end