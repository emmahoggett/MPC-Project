function Xf = terminal_set(mpc, H, h , M, m)
%TERMINAL_SET Compute the maximal invariant set
%   - mpc: MPC controller
%   - H & h: input constraints
%   - M & m: state constraints

      % Problem parameters
      %%% Tuning parameters
      Q = mpc.C'*mpc.C;
      R = 1;
      
      % Compute LQR for unconstrained system
      [K,~,~] = dlqr(mpc.A, mpc.B, Q, R);
      K = - K; % Note that matlab defines K as -K
      
      % Compute the maximal invariant set in closed loop
      Acl = mpc.A+mpc.B*K;
      if (nargin <=3)
        Xf = Polyhedron([H*K],[h]);
      else 
          Xf = Polyhedron([M; H*K],[m;h]);
      end
      
      while 1
          Xfprev = Xf;
          F = Xf.A; f = Xf.b;
          Xf =  Polyhedron([F; F*Acl], [f;f]);
          if Xf == Xfprev, break; end  
      end
      
end

