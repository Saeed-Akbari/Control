% Design a controller for a robot manipulator
clear;
clc;

% Set the initial condition for the state and integration interval
y0 = [0;0;0;0;1+rand(9,1)]; % initial condition
t0 = 1;     % initial time
tf = 150;    % final time
tspan = [t0 tf];

% Set parameters to calculate dynamics
%p1 = 3.473;
%p2 = 0.196;
%p3 = 0.242;
%fd1 = 5.3;
%fd2 = 1.1;
%fs1 = 8.45;
%fs2 = 2.35;
%s1 = 0.5;
%s2 = 0.25;

p1 = 5.473;
p2 = 2.196;
p3 = 1.242;
fd1 = 3.3;
fd2 = 2.1;
fs1 = 3.45;
fs2 = 7.35;
s1 = 1.5;
s2 = 0.75;
P.q_d = [pi ; pi/2];  % Target point

P.p1 = p1*(1+rand(1));
P.p2 = p2*(1+rand(1));
P.p3 = p3*(1+rand(1));
P.fd1 = fd1*(1+rand(1));
P.fd2 = fd2*(1+rand(1));
P.fs1 = fs1*(1+rand(1));
P.fs2 = fs2*(1+rand(1));
P.s1 = s1*(1+rand(1));
P.s2 = s2*(1+rand(1));

% Set control gains
P.alpha = 1; % Change this to suit your needs

% Solve the ode using 'ode45'.
% x = [q; q_dot]
% y = [q; q_dot; theta_hat_dot]
[t,y] = ode45(@(t,y) closedLoopDynamics(t,y,P), tspan, y0);

% Plotting the results
figure(1)
q1 = y(:,1);
q2 = y(:,2);
plot(t,q1,t,q2)
title(append('q'))
xlabel('Time t');
legend('q_1','q_2')

figure(2)
q_dot1 = y(:,3);
q_dot2 = y(:,4);
plot(t,q_dot1,t,q_dot2)
title(append('q_{dot}'))
xlabel('Time t');
legend('q_{dot_1}','q_{dot_2}')

figure(3)
theta_hat8 = y(:,12);
theta_hat9 = y(:,13);
plot(t,theta_hat8,t,theta_hat9)
title(append('Theta estimation'))
xlabel('Time t');
legend('s_1','s_2')


figure(4)
theta_hat1 = y(:,5);
theta_hat2 = y(:,6);
theta_hat3 = y(:,7);
theta_hat4 = y(:,8);
theta_hat5 = y(:,9);
theta_hat6 = y(:,10);
theta_hat7 = y(:,11);
plot(t,theta_hat1,t,theta_hat2,t,theta_hat3,t,theta_hat4,t,theta_hat5,...
    t,theta_hat6,t,theta_hat7,t,theta_hat8,t,theta_hat9)
title(append('Theta estimation'))
xlabel('Time t');
%ylabel('theta estimation');
legend('p1','p2','p3','fd_1','fd_2','fs_1','fs_2')

% FUNCTIONS
%

function y_dot = closedLoopDynamics(t, y, P)
    % This function "simulates" the behavior of a two-link robot under the
    % controller given by the 'control' function.
    x_dot = openLoopDynamics(t, y, control(t, y, P), P);
    theta_hat_dot = updateLaw(t, y, P);
    y_dot = [x_dot;theta_hat_dot];
end

% This is the RLC circuit. In a real experiment, this is replaced by real
% hardware
function x_dot = openLoopDynamics(t, x, u, P)
   x_dot(1,:) = x(3);
   x_dot(2,:) = x(4);
   M = MCal(x(1:2), P);
   Vm = VmCal(x(1:2), x(3:4), P);
   Fd = FdCal(P);
   Fs = FsCal(x(3:4), P);
   S = SCal(P);
   x_dot(3:4,:) = M \ (u - S*x(1:2) - Fd*x(3:4) - Fs - Vm*x(3:4));
end

% Control function, y = [q; q_dot; theta_hat_dot]
function u = control(t, y, P)
    x = y(1:4,:);
    theta_hat = y(5:end,:);
    q = x(1:2);
    q_dot = x(3:4);
    e = P.q_d - q;
    Y1 = Y1vec(q, q_dot, P);
    Y2 = Y2vec(q, q_dot, P);
    u = (Y1 + Y2) * theta_hat + e;
end

% Parameter update law, y = [q; q_dot; theta_hat_dot]
function theta_hat_dot = updateLaw(t, y, P)
    x = y(1:4,:);
    q = x(1:2);
    q_dot = x(3:4);
    e = P.q_d - q;
    e_dot = - q_dot;
    r = e_dot + P.alpha * e;
    Y1 = Y1vec(q, q_dot, P);
    Y2 = Y2vec(q, q_dot, P);
    theta_hat_dot = (transpose(Y1) + transpose(Y2)) * r;
end


% Calculate Fs matrix
function Marr = MCal(q, P)
    Marr = [P.p1 + 2*P.p3*cos(q(2)) , P.p2 + P.p3*cos(q(2)) ;...
            P.p2 + P.p3*cos(q(2)) , P.p2];
end

% Calculate Fs matrix
function Vm = VmCal(q, q_dot, P)
    Vm = [P.p3*sin(q(2))*q_dot(2) , -P.p3*sin(q(2))*(q_dot(1)+q_dot(2))...
        ; P.p3*sin(q(2))*q_dot(1) , 0];
end

% Calculate Fd matrix
function Fd = FdCal(P)
    Fd = [P.fd1 , 0 ; 0 , P.fd2];
end

% Calculate Fs matrix
function Fs = FsCal(q_dot, P)
    Fs = [P.fs1 * tanh(q_dot(1)) ; P.fs2 * tanh(q_dot(2))];
end

% Calculate S matrix
function Sarr = SCal(P)
    Sarr = [P.s1 , 0 ; 0 , P.s2];
end

% Calculate Y1 vector
function Y = Y1vec(q, q_dot, P)
    Y = [-P.alpha*q_dot(1) , -P.alpha*q_dot(2) , -P.alpha*cos(q(2))*...
        (2*q_dot(1)+q_dot(2)) + sin(q(2))*q_dot(1)^2 ,...
        q_dot(1) , 0 , tanh(q_dot(1)), 0, q(1) , 0 ; ...
        0 , -P.alpha*(q_dot(1) + q_dot(2)) , ...
        -q_dot(1)*(P.alpha*cos(q(2)) - sin(q(2))*q_dot(1)) , 0 ,...
        -q_dot(2) , 0 , tanh(q_dot(2)) , 0 , q(2)];
end

% Calculate Y2 vector
function Y = Y2vec(q, q_dot, P)
    Y = [P.alpha*q_dot(1) , P.alpha*q_dot(2) , P.alpha*cos(q(2))*...
        (2*q_dot(1)+q_dot(2)) + sin(q(2))*q_dot(2)*...
        (2*(P.alpha*(P.q_d(1)-q(1))-q_dot(1)) + ...
        (P.alpha*(P.q_d(2)-q(2)) - q_dot(2))) ,...
        0 , 0 , 0 , 0 , 0 , 0 ;
        0 , P.alpha*(q_dot(1)+q_dot(2)) , P.alpha*cos(q(2))*q_dot(1) + ...
        sin(q(2))*q_dot(2)*(P.alpha*(P.q_d(1)-q(1))-q_dot(1)) ,...
        0 , 0 , 0 , 0 , 0 , 0];
    Y = -0.5 * Y;
end