% Design a controller for a RLC circuit
clear;
clc;

% Set the initial condition for the state and integration interval
y0 = [0;0;1+rand(3,1)]; % initial condition
t0 = 1;     % initial time
tf = 12;    % final time
tspan = [t0 tf];

% Set parameters to calculate dynamics
L = 0.2;
C = 2;
R = 0.5;
P.L = L+0.1*L*rand(1);
P.C = C+0.1*C*rand(1);
P.R = R+0.1*R*rand(1);
P.q_d = 5;  % Target point

% Set control gains
P.alpha = 1; % Change this to suit your needs

% Solve the ode using 'ode45'.
% x = [q; q_dot]
% y = [q; q_dot; theta_hat_dot]
[t,y] = ode45(@(t,y) closedLoopDynamics(t,y,P), tspan, y0);


% Plotting the results
figure(1)
q = y(:,1);
q_dot = y(:,2);
plot(t,q,t,q_dot)
xlabel('Time t');
legend('Charge','Current')

figure(2)
theta_hat1 = y(:,3);
theta_hat2 = y(:,4);
theta_hat3 = y(:,5);
plot(t,theta_hat1,t,theta_hat2,t,theta_hat3)
title(append('Theta estimation'))
xlabel('Time t');
legend('1/C','R','L')

% FUNCTIONS
%

function y_dot = closedLoopDynamics(t, y, P)
    % This function "simulates" the behavior of a RLC circuit under the
    % controller given by the 'control' function.
    x_dot = openLoopDynamics(t, y, control(t, y, P), P);
    theta_hat_dot = updateLaw(t, y, P);
    y_dot = [x_dot;transpose(theta_hat_dot)];
end

% This is the RLC circuit. In a real experiment, this is replaced by real
% hardware
function x_dot = openLoopDynamics(t, x, u, P)
   x_dot(1,:) = x(2);
   x_dot(2,:) = (1/P.L)*(u - P.R*x(2) - (1/P.C)*x(1));
end

% Control function, y = [q; q_dot; theta_hat_dot]
function u = control(t, y, P)
    x = y(1:2,:);
    theta_hat = y(3:end,:);
    q = x(1);
    q_dot = x(2);
    e = P.q_d - q;
    Y = Yvec(q, q_dot, P);
    u = Y * theta_hat + e;
end

% Parameter update law, y = [q; q_dot; theta_hat_dot]
function theta_hat_dot = updateLaw(t, y, P)
    x = y(1:2,:);
    q = x(1);
    q_dot = x(2);
    e = P.q_d - q;
    e_dot = - q_dot;
    r = e_dot + P.alpha * e;
    Y = Yvec(q, q_dot, P);
    theta_hat_dot = r * Y;
end


% Calculate Y vector
function Y = Yvec(q, q_dot, P)
    Y = [q , q_dot , -P.alpha * q_dot];
end