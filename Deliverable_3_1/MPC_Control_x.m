classdef MPC_Control_x < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 40;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
      
      % Problem parameters
      %%% Tuning parameters
      Q = mpc.C'*mpc.C;
      R = 1;
      
      %%% Constraints -0.3 <= M_beta <= 0.3
      h = [0.3 0.3]'; 
      H = [1 -1]';
      
      %%% Constraints  |beta| <= 0.035
      M = [0 1 0 0;0 -1 0 0];
      m = [0.035 0.035]';
      
      % Compute LQR for unconstrained system
      [K,P,~] = dlqr(mpc.A, mpc.B, Q, R);
      K = - K; % Note that matlab defines K as -K
      
      %  ----------- Maximal invariant set -----------
      Acl = mpc.A+mpc.B*K;
      Xf = Polyhedron([M; H*K],[m;h]);
      while 1
          Xfprev = Xf;
          F = Xf.A; f = Xf.b;
          Xf =  Polyhedron([F; F*Acl], [f;f]);
          if Xf == Xfprev, break; end  
      end
      F = Xf.A; f = Xf.b;
      

      
      % ----------- Constraints and objective -----------
      con = (x(:,2) == mpc.A*x(:,1) + mpc.B*u(:,1)) + (H*u(:,1) <= h);
      obj = u(:,1)'*R*u(:,1);
      
      for i = 2:N-1
        con = con + (x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i));
        con = con + (H*u(:,i) <= h) + (M*x(:,i)<=m);
        obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
      end
      obj = obj + x(:,N)'*P*x(:,N);
      con = con + (F*x(:,N) <= f);
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      obj = 0;
      
      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
