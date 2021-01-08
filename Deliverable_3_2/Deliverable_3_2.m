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
[sol.x, solx.u] = computeSolution(mpc_x, x0, x_position_reference);

% ----------- Design MPC controller y -----------
mpc_y = MPC_Control_y(sys_y, Ts);
% Initial condition and reference
y0=[0;0;0;0]; y_position_reference= [0;0;0;-2];
[sol.y, soly.u] = computeSolution(mpc_y, y0, y_position_reference);


% ----------- Design MPC controller z -----------
mpc_z = MPC_Control_z(sys_z, Ts);
% Initial condition and reference
z0=[0;0]; z_position_reference= [0;-2];
[sol.z, solz.u] = computeSolution(mpc_z, z0, z_position_reference);

% ----------- Design MPC controller yaw -----------
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
% Initial condition and reference
yaw0=[0;0]; yaw_position_reference = [0;pi/4];
[sol.yaw, solyaw.u] = computeSolution(mpc_yaw, yaw0, yaw_position_reference);

% simulating closed loop    
% Plotting the results
% ------------ along x, y & z-direction ------------ 
figure(9)
hold on; grid on;

o = ones(1,size(sol.x,2));

subplot(3,1,1)
hold on; grid on;
plot(0:Ts:Ts*(length(sol.x(4,:))-1),sol.x(4,:),'markersize',20,'linewidth',2, 'DisplayName','x');
plot(0:Ts:Ts*(length(sol.y(4,:))-1),sol.y(4,:),'markersize',20,'linewidth',2, 'DisplayName','y');
plot(0:Ts:Ts*(length(sol.z(2,:))-1),sol.z(2,:),'markersize',20,'linewidth',2, 'DisplayName','z');
ylabel('Position[m]')
xlabel('Time[s]')


subplot(3,1,2)
hold on; grid on;
plot(0:Ts:Ts*(length(sol.x(3,:))-1),sol.x(3,:),'markersize',20,'linewidth',2, 'DisplayName','x');
plot(0:Ts:Ts*(length(sol.y(3,:))-1),sol.y(3,:),'markersize',20,'linewidth',2,  'DisplayName','y');
plot(0:Ts:Ts*(length(sol.z(1,:))-1),sol.z(1,:),'markersize',20,'linewidth',2,  'DisplayName','z');
ylabel('Velocity[m/s]')
xlabel('Time[s]')
subplot(3,1,3)
hold on; grid on;
plot(0:Ts:Ts*(length(solx.u(1,:))-1),solx.u(1,:),'markersize',20,'linewidth',2,'DisplayName','x');
plot(0:Ts:Ts*(length(soly.u(1,:))-1),soly.u(1,:),'markersize',20,'linewidth',2, 'DisplayName','y');
plot(0:Ts:Ts*(length(solz.u(1,:))-1),solz.u(1,:),'markersize',20,'linewidth',2,  'DisplayName','z');
ylabel('Input u')
xlabel('Time[s]')

% ------------ along yaw-direction ------------ 
figure(10)
hold on; grid on;

o = ones(1,size(sol.x,2));

subplot(3,1,1)
hold on; grid on;
plot(0:Ts:Ts*(length(sol.yaw(2,:))-1),sol.yaw(2,:),'markersize',20,'linewidth',2);
ylabel('Angle[rad]')
xlabel('Time[s]')


subplot(3,1,2)
hold on; grid on;
plot(0:Ts:Ts*(length(sol.yaw(1,:))-1),sol.yaw(1,:),'markersize',20,'linewidth',2);
ylabel('Velocity[rad/s]')
xlabel('Time[s]')

subplot(3,1,3)
hold on; grid on;
plot(0:Ts:Ts*(length(solyaw.u(1,:))-1),solyaw.u(1,:),'markersize',20,'linewidth',2);
ylabel('Input u')
xlabel('Time[s]')

