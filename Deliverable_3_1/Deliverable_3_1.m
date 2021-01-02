%---------------------------------------------
%-------------- Deliverable 3.1 --------------
%---------------------------------------------

% Project : Quadcopter control
% Authors : Balestrini, Durand, Hoggett
% 31 dec. 2020

clc
clear
close all

Ts      = 1/5;
quad    = Quad(Ts);
[xs, us]= quad.trim();
sys     = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%%% Check is sys_x, sys_y, sys_z, sys_yaw are reachable and observable
observ_reach(sys_x, 'x');
observ_reach(sys_y, 'y');
observ_reach(sys_z, 'z');
observ_reach(sys_yaw, 'yaw');

%% Design MPC controller

% ------------ x-direction ------------ 
mpc_x = MPC_Control_x(sys_x, Ts);
x0 = [0 0 0 2]';
[sol.x] = computeSolution(mpc_x, x0);


% ------------ y-direction ------------ 
mpc_y = MPC_Control_y(sys_y, Ts);
y0 = [0 0 0 2]';
[sol.y] = computeSolution(mpc_y, y0);

% ------------ z-direction ------------ 
mpc_z = MPC_Control_z(sys_z, Ts);
z0 = [0 2]';
[sol.z] = computeSolution(mpc_z, z0);

% ------------ yaw-direction ------------ 
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
yaw0 = [0 pi/4]';
[sol.yaw] = computeSolution(mpc_yaw, yaw0);

  
% Plotting the results
% ------------ along x, y & z-direction ------------ 
figure(5)
hold on; grid on;
plot((0:size(sol.x,2)-1)*Ts,sol.x(4,:));
plot((0:size(sol.z,2)-1)*Ts, sol.z(2,:));
plot((0:size(sol.y,2)-1)*Ts,sol.y(4,:));
plot((0:size(sol.x,2)-1)*Ts, ones(size(sol.x,2),1)*0.04,'-.','color', ...
    [0.3010 0.7450 0.9330]);
plot((0:size(sol.x,2)-1)*Ts, -ones(size(sol.x,2),1)*0.04,'-.','color', ...
    [0.3010 0.7450 0.9330]);
ylabel('Position[m]')
xlabel('Time[s]')

% ------------ along yaw-direction ------------ 
figure(6)
hold on; grid on;
plot((0:size(sol.yaw,2)-1)*Ts,sol.yaw(2,:));
plot((0:size(sol.yaw,2)-1)*Ts, ones(size(sol.yaw,2),1)*0.02*pi/4,'-.',...
    'color', [0.3010 0.7450 0.9330]);
plot((0:size(sol.yaw,2)-1)*Ts, -ones(size(sol.yaw,2),1)*0.02*pi/4,'-.',...
    'color', [0.3010 0.7450 0.9330]);
ylabel('Angle[rad]')
xlabel('Time[s]')
