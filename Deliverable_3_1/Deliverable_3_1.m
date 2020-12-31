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
