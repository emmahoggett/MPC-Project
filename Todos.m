clear, clc

addpath('src')

%% Todo 1.2 %%

quad = Quad();
Tf = 1.0; %Time to simulate for

x0 = zeros (12,1);                              % Initial state
u = [0 1.5 1 1.5]';                             % Input to apply
sim = ode45(@(t, x) quad.f(x,u), [0, Tf], x0);  % Solve the system ODE
quad.plot(sim, u);                              % Plot the result


%% Todo 2.1 %%

quad = Quad();

[xs, us] = quad.trim()              % Compute steady state for which 0 = f(xs, us)
sys = quad.linearize(xs, us)        % Linearize the nonlinear model

%% Todo 2.2 %%

sys_transformed = sys * inv(quad.T) % New system is A * x + B * inv(T) * v

%% Todo 2.3 %%

[sys x, sys y, sys z, sys yaw] = quad.decompose(sys, xs, us)