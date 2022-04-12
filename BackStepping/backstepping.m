% Set the initial condition for the state and the time interval for integration
clear;
clc;

t0 = 0;
tf = 10;

z0 = [5;5;5];
tspan = [t0 tf];

% Solve the ode using 'ode45'.
[t,z] = ode45(@(t,z) closedLoopDynamics(z), tspan, z0);


% Plotting the results
figure(1)
plot(t,z(:,1),'-',t,z(:,2),t,z(:,3),'--')
xlabel('Time t');
legend('x1','z2','z3')


% FUNCTIONS

function z_dot = closedLoopDynamics(z)
    z_dot = openLoopDynamics(z, control(z));
end


function z_dot = openLoopDynamics(z, u)
    z_dot = zeros(3,1);
    z_dot(1) = -z(2)-z(1);
    z_dot(2) = z(1)-z(2)+z(3);
    z_dot(3) = -z(2)-z(3);
end

function u = control(z)
    u = (1/(2 + sin(z(1))))*(3*z(1)^6 + ...
  z(1)^7 + (1 + sec(z(1))^2 + tanh(z(1)))*z(2) + 4 *z(2)^2 + ...
  z(2)^3 + z(1)^5*(8 + 3*z(2)) + z(1)^4 *(9 + 11 *z(2)) + ...
  z(1)^3 *(2 + 21*z(2) + 3*z(2)^2 - 2*z(3)) + ...
  z(1)^2 *(1 + tanh(z(1)) + 13 *z(2) + 9*z(2)^2 - 2*z(3)) + ...
  z(1)*(-2 + sec(z(1))^2 + tanh(z(1)) + 9*z(2)^2 + z(2)^3 + ...
     z(2)*(9 - 2*z(3)) - 2*z(3)) - 3*z(3));
end


