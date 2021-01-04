classdef MPC_Control_yaw < MPC_Control
  
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
      N = 20;
      
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
      Q(1,1) = 0.1;
      R = 0.1;
      
      
      %%% Constraints -0.2 <= M_yaw <= 0.2
      h = [0.2 0.2]'; 
      H = [1 -1]';
      
      % Compute LQR for unconstrained system
      [~,P,~] = dlqr(mpc.A, mpc.B, Q, R);
      
      
      % Constraints and objective
      con = (x(:,2) == mpc.A*x(:,1) + mpc.B*u(:,1)) + (H*(u(:,1)-us(:,1)) <= h - H*us(:,1));
      obj = (u(:,1)-us(:,1))'*R*(u(:,1)-us(:,1));
     
      for i = 2:N-1
        con = con + (x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i));
        con = con + (H*(u(:,i)-us(:,1)) <= h - H*us(:,1));
        obj = obj + (x(:,i)-xs(:,1))'*Q*(x(:,i)-xs(:,1)) + (u(:,i)-us(:,1))'*R*(u(:,i)-us(:,1));
      end
      obj = obj + (x(:,N)-xs(:,1))'*P*(x(:,N)-xs(:,1));
      
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
      
      % Problem parameters
      %%% Tuning parameters
      R = 0.1;
      
      %%% Constraints -0.2 <= M_yaw <= 0.2
      h = [0.2 0.2]'; 
      H = [1 -1]';
      
      % Constraints and objective
      con = (xs == mpc.A*xs + mpc.B*us) + (H*us<=h) + (ref == mpc.C*xs);
      obj = us'*R*us;
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
