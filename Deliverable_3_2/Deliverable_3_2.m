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

% ----------- Design MPC controller x -----------
mpc_x = MPC_Control_x(sys_x, Ts);
% Initial condition and reference
x0=[0;0;0;0]; x_position_reference= [0;0;0;-2];
[sol.x] = computeSolution(mpc_x, x0, x_position_reference);

% ----------- Design MPC controller y -----------
mpc_y = MPC_Control_y(sys_y, Ts);
% Initial condition and reference
y0=[0;0;0;0]; y_position_reference= [0;0;0;-2];
[sol.y] = computeSolution(mpc_y, y0, y_position_reference);


% ----------- Design MPC controller z -----------
mpc_z = MPC_Control_z(sys_z, Ts);
% Initial condition and reference
z0=[0;0]; z_position_reference= [0;-2];
[sol.z] = computeSolution(mpc_z, z0, z_position_reference);

% ----------- Design MPC controller yaw -----------
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
% Initial condition and reference
yaw0=[0;0]; yaw_position_reference = [0;-pi/4];
[sol.yaw] = computeSolution(mpc_yaw, yaw0, yaw_position_reference);

% simulating closed loop    
% Plotting the results
figure(1)
hold on; grid on;
plot((0:size(sol.x,2)-1)*Ts,sol.x(4,:));
plot((0:size(sol.z,2)-1)*Ts, sol.z(2,:));
plot((0:size(sol.y,2)-1)*Ts,sol.y(4,:));
plot((0:45)*Ts, -ones(46,1)*2.04,'-.','color', [0.3010 0.7450 0.9330]);
plot((0:45)*Ts, -ones(46,1)*1.96,'-.','color', [0.3010 0.7450 0.9330]);
ylabel('Position[m]')
xlabel('Time[s]')


figure(2)
hold on; grid on;
plot((0:size(sol.yaw,2)-1)*Ts,sol.yaw(2,:));
plot((0:25)*Ts, -ones(26,1)*pi/4*(1-0.02),'-.','color', [0.3010 0.7450 0.9330]);
plot((0:25)*Ts, -ones(26,1)*pi/4*(1+0.02),'-.','color', [0.3010 0.7450 0.9330]);
ylabel('Angle[rad]')
xlabel('Time[s]')
