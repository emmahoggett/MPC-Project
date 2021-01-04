function plot_graph
%PLOT_GRAPH Plot the maximal invariant set in each direction


    Ts      = 1/5;
    quad    = Quad(Ts);
    [xs, us]= quad.trim();
    sys     = quad.linearize(xs, us);
    [sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

    %% --------- MPC Control x Invariant set for x ---------
    mpc_x = MPC_Control_x(sys_x, Ts);
    %%% Constraints -0.3 <= M_beta <= 0.3
    h_x = [0.3 0.3]'; H_x = [1 -1]';
    %%% Constraints  |beta| <= 0.035
    M_x = [0 1 0 0;0 -1 0 0]; m_x = [0.035 0.035]';
    Xf_x = terminal_set(mpc_x, H_x, h_x, M_x, m_x);
    x0 = [0 0 0 2]';
    [sol.x, solu] = computeSolution(mpc_x, x0);

    % Plot the maximal invariant set
    figure(1)
    subplot(3,1,1)
    Xf_x.projection(1:2).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880]);
    xlabel('$\dot{\beta}$', 'Interpreter','latex')
    ylabel('$\beta$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    subplot(3,1,2)
    Xf_x.projection(2:3).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880]);
    xlabel('$\beta$', 'Interpreter','latex')
    ylabel('$\dot{x}$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    subplot(3,1,3)
    Xf_x.projection(3:4).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880]);
    xlabel('$\dot{x}$', 'Interpreter','latex')
    ylabel('$x$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    figure(2)
    hold on
    Xf_x.projection(3:4).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880], 'DisplayName','X_\infty');
    plot(sol.x(3,:),sol.x(4,:),'DisplayName','trajectory');
    plot(sol.x(3,end),sol.x(4,end), 'x', 'DisplayName','final state');
    plot(sol.x(3,40),sol.x(4,40), 'x', 'DisplayName','8[s]');
    xlabel('$\dot{x}$', 'Interpreter','latex')
    ylabel('$x$', 'Interpreter','latex')
    set(gca,'FontSize',20)
    
    % simulating closed loop    
    % Plotting the results
    figure (11)
    hold on; grid on;

    o = ones(1,size(sol.x,2));

    subplot(3,1,1)
    hold on; grid on;
    plot(0:Ts:Ts*(length(sol.x(4,:))-1),sol.x(4,:),'-k','markersize',20,'linewidth',2);
    ylabel('x-Position')

    subplot(3,1,2)
    hold on; grid on;
    plot(0:Ts:Ts*(length(sol.x(3,:))-1),sol.x(3,:),'-k','markersize',20,'linewidth',2);
    ylabel('x-Velocity')

    subplot(3,1,3)
    hold on; grid on;
    plot(0:Ts:Ts*(length(solu(1,:))-1),solu(1,:),'-k','markersize',20,'linewidth',2);
    ylabel('input u')
    %% --------- MPC Control x Invariant set for y ---------
    mpc_y = MPC_Control_y(sys_y, Ts);
    %%% Constraints -0.3 <= M_alpha <= 0.3
    h_y = [0.3 0.3]'; H_y = [1 -1]';
    %%% Constraints  |alpha| <= 0.035
    M_y = [0 1 0 0;0 -1 0 0]; m_y = [0.035 0.035]';
    Xf_y = terminal_set(mpc_y, H_y, h_y, M_y, m_y);
    y0 = [0 0 0 2]';
    [sol.y,~] = computeSolution(mpc_y, y0);

    % Plot the maximal invariant set
    figure(3)
    subplot(3,1,1)
    Xf_y.projection(1:2).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880]);
    xlabel('$\dot{\alpha}$', 'Interpreter','latex')
    ylabel('$\alpha$', 'Interpreter','latex')
    set(gca,'FontSize',20)
    subplot(3,1,2)
    Xf_y.projection(2:3).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880]);
    xlabel('$\alpha$', 'Interpreter','latex')
    ylabel('$\dot{y}$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    subplot(3,1,3)
    Xf_y.projection(3:4).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880]);
    xlabel('$\dot{y}$', 'Interpreter','latex')
    ylabel('$y$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    figure(4)
    hold on
    Xf_x.projection(3:4).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880], 'DisplayName','X_\infty');
    plot(sol.y(3,:),sol.y(4,:),'DisplayName','trajectory');
    plot(sol.y(3,end),sol.y(4,end), 'x', 'DisplayName','final state');
    plot(sol.y(3,40),sol.y(4,40), 'x', 'DisplayName','8[s]');
    xlabel('$\dot{y}$', 'Interpreter','latex')
    ylabel('$y$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    %% --------- MPC Control x Invariant set for z ---------
    mpc_z = MPC_Control_z(sys_z, Ts);
    %%% Constraints -0.2 <= F <= 0.3
    h_z = [0.3 0.2]'; 
    H_z = [1 -1]';
    Xf_z = terminal_set(mpc_z, H_z, h_z);
    z0 = [0 2]';
    [sol.z, ~] = computeSolution(mpc_z, z0);

    figure(5)
    Xf_z.plot('alpha',0.5,'color',  [0.4660 0.6740 0.1880]);
    xlabel('$\dot{z}$', 'Interpreter','latex')
    ylabel('$z$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    figure(6)
    hold on
    Xf_z.projection(1:2).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880], 'DisplayName','X_{\infty}');
    plot(sol.z(1,:),sol.z(2,:),'DisplayName','trajectory');
    plot(sol.z(1,end),sol.z(2,end), 'x', 'DisplayName','final state');
    xlabel('$\dot{z}$', 'Interpreter','latex')
    ylabel('$z$', 'Interpreter','latex')
    set(gca,'FontSize',20)



    %% --------- MPC Control x Invariant set for yaw ---------
    mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
    h_yaw = [0.2 0.2]';  H_yaw = [1 -1]';
    Xf_yaw = terminal_set(mpc_yaw, H_yaw, h_yaw);
    yaw0 = [0 pi/4]';
    [sol.yaw, ~] = computeSolution(mpc_yaw, yaw0);

    figure(7)
    Xf_yaw.projection(1:2).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880]);
    xlabel('$\dot{\gamma}$', 'Interpreter','latex')
    ylabel('$\gamma$', 'Interpreter','latex')
    set(gca,'FontSize',20)

    figure(8)
    hold on
    Xf_yaw.projection(1:2).plot('alpha',0.5,'color', [0.4660 0.6740 0.1880], 'DisplayName','X_{\infty}');
    plot(sol.yaw(1,:),sol.yaw(2,:),'DisplayName','trajectory');
    plot(sol.yaw(1,end),sol.yaw(2,end), 'x', 'DisplayName','final state');
    xlabel('$\dot{\gamma}$', 'Interpreter','latex')
    ylabel('$\gamma$', 'Interpreter','latex')
    set(gca,'FontSize',20)


end

