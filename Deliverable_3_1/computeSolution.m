function [sol] = computeSolution(mpc_sys, x0)
    % Compute the displacement along a direction of the quadcopter
    %   - mpc_sys : MPC system -
    %   - x0 : initial position -
    %   - xref : desired position -
    
    sol.x(:,1) = x0;
    i = 1;
    
    while norm(sol.x(:,end)) > 1e-2% Simulate until convergence
        % Solve MPC problem for current state
        uopt = mpc_sys.get_u(sol.x(:,i));

        % Extract the optimal input
        sol.u(:,i) = uopt;

        % Apply the optimal input to the system
        sol.x(:,i+1) = mpc_sys.A*sol.x(:,i) + mpc_sys.B*sol.u(:,i);
        i = i + 1;
        
    end
    sol = sol.x;
end
