clc
clear
close all

Ts      = 1/5;
quad    = Quad(Ts);
[xs, us]= quad.trim();
sys     = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
Xf = terminal_set(mpc_yaw);

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


      
      
figure(1)
hold on
plot(Xf.projection(1:2),'color', [0.4660 0.6740 0.1880]);
plot(sol.yaw(1,:),sol.yaw(2,:));
plot(sol.yaw(1,end),sol.yaw(1,end), 'x');

xlabel('$\dot{\gamma}$', 'Interpreter','latex','FontSize',15)
ylabel('$\gamma$', 'Interpreter','latex','FontSize',15)



function Xf = terminal_set(mpc)

       % Problem parameters
      %%% Tuning parameters
      Q = mpc.C'*mpc.C;
      R = 1;
      
      
      %%% Constraints -0.2 <= M_yaw <= 0.2
      h = [0.2 0.2]'; 
      H = [1 -1]';
      
      % Compute LQR for unconstrained system
      [K,P,~] = dlqr(mpc.A, mpc.B, Q, R);
      K = - K; % Note that matlab defines K as -K
      
      % Compute the maximal invariant set in closed loop
      Acl = mpc.A+mpc.B*K;
      Xf = Polyhedron([H*K],[h]);
      while 1
          Xfprev = Xf;
          F = Xf.A; f = Xf.b;
          Xf =  Polyhedron([F; F*Acl], [f;f]);
          if Xf == Xfprev, break; end  
      end
      
end