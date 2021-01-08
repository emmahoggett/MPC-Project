function ctrl= ctrl_NMPC(quad)

    import casadi.*

    opti = casadi.Opti();   % Optimization problem 
    N = 20;                 % MPC horizon [SET THIS VARIABLE]

    % ---- decision variables ------
    X = opti.variable(12,N+1); % state trajectory variables
    U = opti.variable(4, N);   % cont@(x,ref)eval_ctrl(x,ref,opti,X0,REF,X,U)rol trajectory (throttle, brake)

    X0 = opti.parameter(12,1); % initial state
    REF = opti.parameter(4,1); % reference position [x,y,z,yaw]

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%%% YOUR CODE HERE %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Continuous-time dynamics
    f=@(x,u) quad.f(x,u);

    % Discrete-time model
    Ts=1/5; 
    f_discrete=@(x,u) RK4(x,u,Ts,f);

    % Objective

     opti.minimize(30*sumsqr(X(10:12,:)-REF(1:3)) +...
                   30*sumsqr(X(6,:)-REF(4)) +...
                    1*sumsqr(X(1:3,:)) +...
                    1*sumsqr(X(4:5,:)) +...
                    5*sumsqr(X(7:8,:)) +...
                   40*sumsqr(X(9,:)) +...
                  0.1*sumsqr(U));

    % Dynamic constraints
    for i=1:N
        opti.subject_to(X(:,i+1)==f_discrete(X(:,i),U(:,i)));
    end

    % ---- path constraints -----------
    opti.subject_to(0 <= U <= 1.5);  % control is limited

    % ---- boundary conditions --------
    opti.subject_to(X(:,1)==X0);   % use initial position

    ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end

function u = eval_ctrl(x, ref, opti, X0, REF, X, U) 
    % ---- Set the initial state and reference ---- 
    opti.set_value(X0, x);
    opti.set_value(REF, ref);

    % ---- Setup solver NLP ------
    ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false); 
    opti.solver('ipopt', ops);

    % ---- Solve the optimization problem ----
    sol = opti.solve();
    assert(sol.stats.success == 1, 'Error computing optimal input');

    u = opti.value(U(:,1));
    
    % Use the current solution to speed up the next optimization
    opti.set_initial(sol.value_variables());
    opti.set_initial(opti.lam_g, sol.value(opti.lam_g)); 
end
