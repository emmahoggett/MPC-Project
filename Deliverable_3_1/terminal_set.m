function Xf = terminal_set(mpc, M, m , H, h)
%TERMINAL_SET Compute the maximal invariant set
%   - mpc: MPC controller
%   - H & h: input constraints
%   - M & m: state constraints

      % Problem parameters
      %%% Tuning parameters
      Q = mpc.C'*mpc.C;
      R = 0.1;
      
      if (nargin <=3)
          Q(1,1) = 0.1;
      else
          Q(1,1) = 0.1; Q(2,2) = 0.1; Q(3,3) = 0.1;
      end
      
      
      % Compute LQR for unconstrained system
      [K,~,~] = dlqr(mpc.A, mpc.B, Q, R);
      K = - K; % Note that matlab defines K as -K
      
      % Compute the maximal invariant set in closed loop
      Acl = mpc.A+mpc.B*K;
      if (nargin <=3)
        Xf = polytope([M*K],[m]);
      else 
          Xf = polytope([H; M*K],[h;m]);
      end
      
      while 1
          prevXf = Xf;
          [T,t] = double(Xf);
          preXf = polytope(T*Acl,t);
          Xf = intersect(Xf, preXf);
          if isequal(prevXf, Xf)
              break
          end
      end
      
end

