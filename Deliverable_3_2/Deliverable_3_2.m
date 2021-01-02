%---------------------------------------------
%-------------- Deliverable 3.2 --------------
%---------------------------------------------

% Project : Quadcopter control
% Authors : Balestrini, Durand, Hoggett
% 31 dec. 2020

clc
clear
close all

Ts      = 1/5;

quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
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

mpc_y = MPC_Control_y(sys_y, Ts);
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
mpc_z = MPC_Control_z(sys_z, Ts);
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
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
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
