% this code is done by PhD candidate Trang NGUYEN, LCIS-Grenoble INP


clear all
close all
clc
% Import Casadi -> this is the modeling language that we use for solving
% optimization problems


import casadi.*
solver = casadi.Opti(); % using Opti class


%% System
b = 0.1;
% Write here the continuous state space of the sysem
A = [0 0; 0 0];
B = [1 0; 0 1];


% Discretization
Ts = 0.01;
Ad = [1 0; 0 1];
Bd = [Ts 0; 0 Ts];


% system dimension
[nx, nu] = size(Bd);
nsi = 3;


% Tuning MPC
Q = [2 0; 0 2]; %cost per stage
R = [5 0; 0 5]; %input cost
%R = Q;
%P = Q;
P = [3 0; 0 3]; %
Npred = 10; % prediction horizon


% Find the gain of the LQR controller using the same tuning of MPC
K_lqr = dlqr(Ad, Bd, Q, R);




%Constraints on angular veloctiy and linear velocity
max_ang_vel = 2.84;
max_lin_vel = 0.22;






%% Initial conditions
x0 = [-1.0; -1.0; pi/2];
z_target = [1; 0.9];


% Simulation time
Tsim = 30;
Nsim = round(Tsim/Ts);


%% MPC definition in CacADi - Define variables, parameters of Casadi
% Using Opti class
solver = casadi.Opti();


% Define variable
% x = solver.parameter(nx+1, Npred+1);
z = solver.variable(nx, Npred+1);
theta_cas = solver.variable(1, Npred + 1);
u = solver.variable(nu, Npred+1);


% Define parameters
z_init = solver.parameter(nx, 1);
theta_cas_init = solver.parameter(1,1);


% Constraint
solver.subject_to(z(:,1) == z_init);
solver.subject_to(theta_cas(1) == theta_cas_init);


for i = 1:Npred
    solver.subject_to(z(:, i+1) == Ad*z(:, i) + Bd*u(:, i));
    c = cos(theta_cas(i));
    s = sin(theta_cas(i));
    T = [c, s;
        -s/b, c/b];
    u_real_cas = T * u(:, i);
    
    solver.subject_to(u_real_cas(1) <= max_lin_vel);
    solver.subject_to(u_real_cas(2) <= max_ang_vel);
    solver.subject_to(u_real_cas(1) >= -max_lin_vel);
    solver.subject_to(u_real_cas(2) >= -max_ang_vel);
    solver.subject_to(theta_cas(i+1) == theta_cas(i) + Ts*u_real_cas(2));
end
objective = 0;


for j = 1:Npred
    objective = objective + (z(:, j) - z_target)'*Q*(z(:, j) - z_target)... 
    + u(:, j)'*R*u(:, j);
end


% add the terminal cost to the objective
objective = objective + (z(:, Npred+1) - z_target)'*P*(z(:, Npred+1) - z_target);
% Optimize the objective
solver.minimize(objective);


% Define the solver
options = struct
options.ipopt.print_level = 0;
options.ipopt.sb = 'yes';
options.print_time = 0;
solver.solver('ipopt', options);


%% Simulation loop of MPC

tic
usim_vir_mpc = zeros(nu, Nsim);
usim_real_mpc = zeros(nu, Nsim);
z_sim_mpc_mpc = zeros(nx, Nsim);
x_sim_mpc = zeros(nx+1, Nsim+1);
x_sim_mpc(:, 1) = x0;


for k = 1:Nsim
    objective = 0;
    theta = x_sim_mpc(3,k);
    z_sim_mpc(:, k) = [x_sim_mpc(1,k) + b*cos(theta);...
                    x_sim_mpc(2,k) + b*sin(theta)];
    solver.set_value(z_init, z_sim_mpc(:,k));
    solver.set_value(theta_cas_init, theta);
    sol = solver.solve();
    usol = sol.value(u);
    usim_vir_mpc(:, k) = usol(:, 1);
    usim_real_mpc(:, k) = [cos(theta), sin(theta);
                   -sin(theta)/b, cos(theta)/b] * usim_vir_mpc(:, k);
    x_sim_mpc(:, k+1) = x_sim_mpc(:, k) + Ts*[usim_real_mpc(1, k) * cos(theta);
                                      usim_real_mpc(1, k) * sin(theta);
                                      usim_real_mpc(2, k)];
end
toc

%% Simulation control loop for LQR


usim_vir_lqr = zeros(nu, Nsim);
usim_real_lqr = zeros(nu, Nsim);
z_sim_lqr = zeros(nx, Nsim);
x_sim_lqr = zeros(nx+1, Nsim+1);
x_sim_lqr(:, 1) = x0;


for k = 1:Nsim
    theta_lqr = x_sim_lqr(3,k);
    z_sim_lqr(:, k) = [x_sim_lqr(1,k) + b*cos(theta_lqr);...
                       x_sim_lqr(2,k) + b*sin(theta_lqr)];
    usim_vir_lqr(:, k) = K_lqr * (z_target - z_sim_lqr(:, k));


    usim_real_lqr(:, k) = [cos(theta_lqr), sin(theta_lqr);
                          -sin(theta_lqr)/b, cos(theta_lqr)/b] * usim_vir_lqr(:, k);


    % Saturation
    usim_real_lqr(1,k) = min( max(usim_real_lqr(1,k), -max_lin_vel), max_lin_vel);
    usim_real_lqr(2,k) = min( max(usim_real_lqr(2,k), -max_ang_vel), max_ang_vel);


    % Update the state
    x_sim_lqr(:, k+1) = x_sim_lqr(:, k) + Ts*[usim_real_lqr(1, k) * cos(theta_lqr);
                                              usim_real_lqr(1, k) * sin(theta_lqr);
                                              usim_real_lqr(2, k)];


end


%% Plotting trajectory of MPC 


close all
frame_total = 250;                   % number of frame for video
st = round(Nsim / frame_total);      % step of frame
cnt = 1;
d = 1.5;                              % limit of frame size


color_bot = [1.0, 0.4, 0.7];           % color of robot


figure
xlim([-1 1]*d)
ylim([-1 1]*d)
xlabel('$x$ (m)', 'interpreter','latex','fontsize',13)
ylabel('$y$ (m)', 'interpreter','latex','fontsize',13)
hold on 
grid on
frid = 0;


while cnt <= Nsim
    frid = frid + 1;
    hold on


    % plot trajectory
    plot(x_sim_mpc(1,1:cnt), x_sim_mpc(2,1:cnt), 'color', 'b', 'LineWidth', 2);


    % plot target
    scatter(z_target(1), z_target(2), 80, 'r', 'filled', 'd');


    % plot robot
    drawWMB(x_sim_mpc(:,cnt), 0.138, 0.178, [0 0 0], color_bot);  % Width, Length of the robot in the plot


    drawnow
    hold off
    Fvid(frid) = getframe(gcf);


    % Fix frame
    clf
    xlim([-1 1]*d)
    ylim([-1 1]*d)
    xlabel('$x$ (m)', 'interpreter','latex','fontsize',13)
    ylabel('$y$ (m)', 'interpreter','latex','fontsize',13)
    grid on
    cnt = cnt + st;
end


% Plot the final frame
frid = frid + 1;
hold on
plot(x_sim_mpc(1,1:cnt), x_sim_mpc(2,1:cnt), 'color', 'b', 'LineWidth', 2);
scatter(z_target(1), z_target(2), 80, 'r', 'filled', 'd');
drawWMB(x_sim_mpc(:,cnt), 0.138, 0.178, [0 0 0], color_bot);
xlabel('$x$ (m)', 'interpreter','latex','fontsize',13);
ylabel('$y$ (m)', 'interpreter','latex','fontsize',13);
title('Trajectory of robot using MPC');
Fvid(frid) = getframe(gcf);




%% Plotting trajectory of LQR 
close all
frame_total = 250;                   % number of frame for video
st = round(Nsim / frame_total);      % step of frame
cnt = 1;
d = 1.5;                              % limit of frame size


color_bot_lqr = [0.2, 0.6, 1];          % color of robot


figure
xlim([-1 1]*d)
ylim([-1 1]*d)
xlabel('$x$ (m)', 'interpreter','latex','fontsize',13)
ylabel('$y$ (m)', 'interpreter','latex','fontsize',13)
hold on 
grid on
frid = 0;


while cnt <= Nsim
    frid = frid + 1;
    hold on


    % plot trajectory
    plot(x_sim_lqr(1,1:cnt), x_sim_lqr(2,1:cnt), 'color', 'k', 'LineWidth', 2);


    % plot target
    scatter(z_target(1), z_target(2), 80, 'r', 'filled', 'd');


    % plot robot
    drawWMB(x_sim_lqr(:,cnt), 0.138, 0.178, [0 0 0], color_bot_lqr);  % W, L 


    drawnow
    hold off
    Fvid(frid) = getframe(gcf);


    % Fix frame
    clf
    xlim([-1 1]*d)
    ylim([-1 1]*d)
    xlabel('$x$ (m)', 'interpreter','latex','fontsize',13)
    ylabel('$y$ (m)', 'interpreter','latex','fontsize',13)
    grid on
    cnt = cnt + st;
end


% Plot the final frame
frid = frid + 1;
hold on
plot(x_sim_lqr(1,1:cnt), x_sim_lqr(2,1:cnt), 'color', 'b', 'LineWidth', 2);
scatter(z_target(1), z_target(2), 80, 'r', 'filled', 'd');
drawWMB(x_sim_lqr(:,cnt), 0.138, 0.178, [0 0 0], color_bot_lqr);
xlabel('$x$ (m)', 'interpreter','latex','fontsize',13)
ylabel('$y$ (m)', 'interpreter','latex','fontsize',13)
title('Trajectory of robot using LQR');
% Fvid(frid) = getframe(gcf);


%% Plotting velocity
time_plot = (1:Nsim)*Ts;
figure;
subplot(2,1,1);
plot(time_plot, usim_real_mpc(1, :), 'b', 'LineWidth',2);
hold on;
plot(time_plot, usim_real_lqr(1, :), ':', 'Color', [0.6, 0.3, 0], 'LineWidth', 2);
yline(0.22, 'r--', 'LineWidth',2);
yline(-0.22, 'r--', 'LineWidth',2);
xlabel('Time (s)');
ylabel('Linear velocity (m/s)');
title('Linear Velocity');
legend('Linear Velocity MPC', 'Linear Velocity LQR', 'Linear velocity limits');
grid on;


subplot(2,1,2);
plot(time_plot, usim_real_mpc(2, :), 'g', 'LineWidth',2);
hold on;
plot(time_plot, usim_real_lqr(2, :), ':', 'Color', [1 0.5 0], 'LineWidth', 2);
yline(2.84, 'r--', 'LineWidth',2);
yline(-2.84, 'r--', 'LineWidth',2);
xlabel('Time(s)');
ylabel('Angular velocity (rad/s)');
title('Angular velocity');
legend('Angular velocity MPC','Angular velocity LQR', 'Angular velocity limits');
grid on;


%% Plotting x,y following time
figure;


subplot(2,1,1);
plot(time_plot, z_sim_mpc(1, :), 'b', 'LineWidth',2);
hold on;
plot(time_plot, z_sim_lqr(1, :), ':', 'Color', [0.6, 0.3, 0], 'LineWidth', 2);
yline(z_target(1), 'r--', 'LineWidth',2);


xlabel('Time (s)');
ylabel('X (m)');
title('X position');
legend('X position MPC', 'X position LQR','X target');


subplot(2,1,2);
plot(time_plot, z_sim_mpc(2, :), 'g', 'LineWidth',2);
hold on;
plot(time_plot, z_sim_lqr(2, :), ':', 'Color', [1 0.5 0], 'LineWidth', 2);
yline(z_target(2), 'r--', 'LineWidth',2);
xlabel('Time(s)');
ylabel('Y (m)');
title('Y position');
legend('Y position MPC', 'Y position LQR', 'Y target');
grid on;