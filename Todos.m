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
s
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us)

%% Todo 3.1 %%
clc
clear
close all

Ts      = 1/5;
quad    = Quad(Ts);
[xs, us]= quad.trim();
sys     = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%%% Check is sys_x, sys_y, sys_z, sys_yaw are reachable and observable
rank(ctrb(sys_x.A, sys_x.B));   % = 4 - full rank -> controllable -> reachable 
rank(obsv(sys_x.A, sys_x.C));   % = 4 - full rank -> observable

rank(ctrb(sys_y.A, sys_y.B));   % = 4 - full rank -> controllable -> reachable 
rank(obsv(sys_y.A, sys_y.C));   % = 4 - full rank -> observable

rank(ctrb(sys_z.A, sys_z.B));   % = 2 - full rank -> controllable -> reachable 
rank(obsv(sys_z.A, sys_z.C));   % = 2 - full rank -> observable

rank(ctrb(sys_yaw.A, sys_yaw.B));   % = 2 - full rank -> controllable -> reachable 
rank(obsv(sys_yaw.A, sys_yaw.C));   % = 2 - full rank -> observable
%% Design MPC controller

%%% x-direction %%%
mpc_x = MPC_Control_x(sys_x, Ts);

x0 = [0 0 0 2]';
sol.x(:,1) = x0;
ix = 1;

while norm(sol.x(:,end)) > 1e-3 % Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_x.get_u(sol.x(:,ix));
    
    % Extract the optimal input
    sol.u(:,ix) = uopt;

    % Apply the optimal input to the system
    sol.x(:,ix+1) = mpc_x.A*sol.x(:,ix) + mpc_x.B*sol.u(:,ix);
    ix = ix + 1;

end


%%% y-direction %%%
mpc_y = MPC_Control_y(sys_y, Ts);

y0 = [0 0 0 2]';
sol.y(:,1) = y0;
iy = 1;

while norm(sol.y(:,end)) > 1e-3 % Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_y.get_u(sol.y(:,iy));
    
    % Extract the optimal input
    sol.u(:,iy) = uopt;

    % Apply the optimal input to the system
    sol.y(:,iy+1) = mpc_y.A*sol.y(:,iy) + mpc_y.B*sol.u(:,iy);
    iy = iy + 1;
end


mpc_z = MPC_Control_z(sys_z, Ts);

z0 = [0 2]';


%z-direction
sol.z(:,1) = z0;
iz = 1;

while norm(sol.z(:,end)) > 1e-3 % Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_z.get_u(sol.z(:,iz));
    
    % Extract the optimal input
    sol.u(:,iz) = uopt;

    % Apply the optimal input to the system
    sol.z(:,iz+1) = mpc_z.A*sol.z(:,iz) + mpc_z.B*sol.u(:,iz);
    iz = iz + 1;

end

% simulating closed loop    
% Plotting the results

figure(5)
hold on; grid on;
plot((0:size(sol.x,2)-1)*Ts,sol.x(4,:));
plot((0:size(sol.z,2)-1)*Ts, sol.z(2,:));
plot((0:size(sol.y,2)-1)*Ts,sol.y(4,:));
plot((0:size(sol.x,2)-1)*Ts, ones(size(sol.x,2),1)*0.04,'-.','color', [0.3010 0.7450 0.9330]);
plot((0:size(sol.x,2)-1)*Ts, -ones(size(sol.x,2),1)*0.04,'-.','color', [0.3010 0.7450 0.9330]);
ylabel('Position[m]')
xlabel('Time[s]')


mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

yaw0 = [0 pi/4]';


%z-direction
sol.yaw(:,1) = yaw0;
iyaw = 1;

while norm(sol.yaw(:,end)) > 1e-3 % Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_yaw.get_u(sol.yaw(:,iyaw));
    
    % Extract the optimal input
    sol.u(:,iyaw) = uopt;

    % Apply the optimal input to the system
    sol.yaw(:,iyaw+1) = mpc_yaw.A*sol.yaw(:,iyaw) + mpc_yaw.B*sol.u(:,iyaw);
    iyaw = iyaw + 1;

end
%%
figure(6)
hold on; grid on;
plot((0:size(sol.yaw,2)-1)*Ts,sol.yaw(2,:));
plot((0:size(sol.yaw,2)-1)*Ts, ones(size(sol.yaw,2),1)*0.02*pi/4,'-.','color', [0.3010 0.7450 0.9330]);
plot((0:size(sol.yaw,2)-1)*Ts, -ones(size(sol.yaw,2),1)*0.02*pi/4,'-.','color', [0.3010 0.7450 0.9330]);
ylabel('Angle[rad]')
xlabel('Time[s]')

%% Todo 3.2 %%
clc
clear
close all

Ts      = 1/5;

quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
% Design MPC controller
mpc_x = MPC_Control_x_32(sys_x, Ts);
% Get control inputs with
x0=[0;0;0;0];
x_position_reference= [0;0;0;-2];

sol.x(:,1) = x0;
i = 1;


while norm(sol.x(:,end)-x_position_reference) > 1e-2% Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_x.get_u(sol.x(:,i),x_position_reference(end));

    % Extract the optimal input
    sol.u(:,i) = uopt;

    % Apply the optimal input to the system
    sol.x(:,i+1) = mpc_x.A*sol.x(:,i) + mpc_x.B*sol.u(:,i);
    i = i + 1;

end

mpc_y = MPC_Control_y_32(sys_y, Ts);
% Get control inputs with
y0=[0;0;0;0];
y_position_reference= [0;0;0;-2];

sol.y(:,1) = y0;
i = 1;


while norm(sol.y(:,end)-y_position_reference) > 1e-2% Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_y.get_u(sol.y(:,i),y_position_reference(end));

    % Extract the optimal input
    sol.u(:,i) = uopt;

    % Apply the optimal input to the system
    sol.y(:,i+1) = mpc_y.A*sol.y(:,i) + mpc_y.B*sol.u(:,i);
    i = i + 1;

end


% Design MPC controller
mpc_z = MPC_Control_z_32(sys_z, Ts);
% Get control inputs with
z0=[0;0];
z_position_reference= [0;-2];

sol.z(:,1) = z0;
iz = 1;


while norm(sol.z(:,end)-z_position_reference) > 1e-2% Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_z.get_u(sol.z(:,iz),z_position_reference(end));

    % Extract the optimal input
    sol.u(:,iz) = uopt;

    % Apply the optimal input to the system
    sol.z(:,iz+1) = mpc_z.A*sol.z(:,iz) + mpc_z.B*sol.u(:,iz);
    iz = iz + 1;

end



% simulating closed loop    
% Plotting the results

figure(7)
hold on; grid on;
plot((0:size(sol.x,2)-1)*Ts,sol.x(4,:));
plot((0:size(sol.z,2)-1)*Ts, sol.z(2,:));
plot((0:size(sol.y,2)-1)*Ts,sol.y(4,:));
plot((0:45)*Ts, -ones(46,1)*2.04,'-.','color', [0.3010 0.7450 0.9330]);
plot((0:45)*Ts, -ones(46,1)*1.96,'-.','color', [0.3010 0.7450 0.9330]);
ylabel('Position[m]')
xlabel('Time[s]')


% Design MPC controller
mpc_yaw = MPC_Control_yaw_32(sys_yaw, Ts);
% Get control inputs with
yaw0=[0;0];
yaw_position_reference= [0;-pi/4];

sol.yaw(:,1) = yaw0;
iyaw = 1;


while norm(sol.yaw(:,end)-yaw_position_reference) > 1e-2% Simulate until convergence
    % Solve MPC problem for current state
    uopt = mpc_yaw.get_u(sol.yaw(:,iyaw),yaw_position_reference(end));

    % Extract the optimal input
    sol.u(:,iyaw) = uopt;

    % Apply the optimal input to the system
    sol.yaw(:,iyaw+1) = mpc_yaw.A*sol.yaw(:,iyaw) + mpc_yaw.B*sol.u(:,iyaw);
    iyaw = iyaw + 1;

end
figure(8)
hold on; grid on;
plot((0:size(sol.yaw,2)-1)*Ts,sol.yaw(2,:));
plot((0:25)*Ts, -ones(26,1)*pi/4*(1-0.02),'-.','color', [0.3010 0.7450 0.9330]);
plot((0:25)*Ts, -ones(26,1)*pi/4*(1+0.02),'-.','color', [0.3010 0.7450 0.9330]);
ylabel('Angle[rad]')
xlabel('Time[s]')


%% Todo 4.1 %%
clc
clear
close all

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
mpc_x = MPC_Control_x_32(sys_x, Ts);
mpc_y = MPC_Control_y_32(sys_y, Ts);
mpc_z = MPC_Control_z_32(sys_z, Ts);
mpc_yaw = MPC_Control_yaw_32(sys_yaw, Ts);

sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
quad.plot(sim);

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