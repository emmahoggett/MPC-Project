clear, clc

addpath('src')
addpath('/home/emma-hoggett/Documents/MATLAB/plugin')

%% Todo 1.2 %%

quad = Quad();
Tf = 1.0; %Time to simulate for

x0 = zeros (12,1);                              % Initial state
u = [0 0 0.5 0]';                               % Input to apply
sim = ode45(@(t, x) quad.f(x,u), [0, Tf], x0);  % Solve the system ODE
quad.plot(sim, u);                              % Plot the result


%% Todo 2.1 %%

quad = Quad();

[xs, us] = quad.trim()              % Compute steady state for which 0 = f(xs, us)
sys = quad.linearize(xs, us)        % Linearize the nonlinear model

%% Todo 2.2 %%

sys_transformed = sys * inv(quad.T) % New system is A * x + B * inv(T) * v

%% Todo 2.3 %%

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us)


%% Todos 5.1 %%
clc
clear
close all

BIAS = -0.1;
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
mpc_x = MPC_Control_x_32(sys_x, Ts);
mpc_y = MPC_Control_y_32(sys_y, Ts);

mpc_z = MPC_Control_z_5(sys_z, Ts);
mpc_yaw = MPC_Control_yaw_32(sys_yaw, Ts);

sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, BIAS);
quad.plot(sim);

%% Todos 6.1 %%
clc
clear
close all

quad = Quad();
CTRL= ctrl_NMPC(quad);

sim = quad.sim(CTRL);
quad.plot(sim);