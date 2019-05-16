% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
% param = compute_controller_base_parameters;
param = compute_controller_base_parameters_jose;
% execute simulation starting from T0_1 using lqr controller with scenario 1
x0_1 = [3 1 0]';
T0_1 = param.T_sp + x0_1;

x0_2 = [-1 -0.1 -4.5]';
T0_2 = param.T_sp + x0_2;

% global T_est
% T_est = T0_1;
% global d_est
% d_est = param.d;
[T,p,t_sim] = simulate_truck(T0_1, @controller_mpc_2_jose, scen1);
% disp('t_sim: ');disp(t_sim);
%%
% param = compute_controller_base_parameters; % get basic controller parameters
% [param.P, ~, ~] = dare(param.A, param.B, param.Q, param.R);
% 
% Ax_cons = [eye(3);-eye(3)];
% bx_cons = [param.Xcons(:,2);-param.Xcons(:,1)];
% Au_cons = [eye(2);-eye(2)];
% bu_cons = [param.Ucons(:,2);-param.Ucons(:,1)];
% 
% N = 30;
% nx = size(param.A,1);
% nu = size(param.B,2);
% 
% U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
% 
% objective = 0;
% constraints = [];
% for k = 1:N-1
%   % system dynamic constraints
%   constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
%   % state constraints
%   constraints = [constraints, Ax_cons*X{k+1} <= bx_cons];
%   % input constraints
%   constraints = [constraints, Au_cons*U{k} <= bu_cons];
%   % objective, sum of stage cost function
%   objective = objective + X{k}' * param.Q * X{k} + U{k}' * param.R * U{k};
% end
% l_f = X{end}'*param.P*X{end};
% objective = objective + l_f;
% 
% x0 = sdpvar(3,1);
% constraints = [constraints, X{1} == x0];
% 
% codeoptions = getOptions('MPC_Controller_1');
% forces_optimizer = optimizerFORCES(constraints, objective, codeoptions,x0,U{1});

% [~,~,~] = simulate_truck(T0_1, @controller_mpc_1_forces, scen1);
% [~,~,~] = simulate_truck(T0_1, @controller_mpc_1, scen1);
% disp('//////////////////////////////////////////////////////////');
% %%
% t_sim_forces_avg = 0;
% for i = 1:50
%     [~, ~,t_sim_forces] = simulate_truck(T0_1, @controller_mpc_1_forces, scen1);
%     t_sim_forces_avg = t_sim_forces_avg + t_sim_forces;
% end
% t_sim_forces_avg = t_sim_forces_avg / 50;
% disp('t_sim_forces: ');disp(t_sim_forces);
%%
% t_sim_avg = 0;
% for i = 1:50
%     [~, ~,t_sim] = simulate_truck(T0_1, @controller_mpc_1, scen1);
%     t_sim_avg = t_sim_avg + t_sim;
% end
% t_sim_avg = t_sim_avg / 50;
% disp('t_sim_avg: ');disp(t_sim_avg);


err = norm(param.T_sp-T(:,30));
if err <= 0.2*norm(x0_2)
    disp('Performance achieved');
else
    disp('Not good enough');
end

%%
% figure;
% subplot(3,1,1);
% plot(1:length(T),T(1,:),'k');
% hold on;
% plot(1:length(T_est),T_est(1,:),'b-.');
% subplot(3,1,2);
% plot(1:length(T),T(2,:),'k');
% hold on;
% plot(1:length(T_est),T_est(2,:),'b-.');
% subplot(3,1,3);
% plot(1:length(T),T(3,:),'k');
% hold on;
% plot(1:length(T_est),T_est(3,:),'b-.');
% %%
% figure;
% subplot(3,1,1);
% plot(1:length(d_est),d_est(1,:),'b-.');
% subplot(3,1,2);
% plot(1:length(d_est),d_est(2,:),'b-.');
% subplot(3,1,3);
% plot(1:length(d_est),d_est(3,:),'b-.');



% [param.P, ~, ~] = dare(param.A, param.B, param.Q, param.R);
% l_f = X{end}'*param.P*X{end};
% p = controller_mpc_1(T0_1);
% x_i = T0_1 - param.T_sp;
% u_i = p - param.p_sp;
% J_MPC1 = x_i'*param.Q*x_i + u_i'*param.R*u_i + l_f; 

%%
% J_inf = 0;
% % [T2, p2] = simulate_truck(T(:,end), @controller_lqr, scen1);
% for i = 1:100
%     u_i = (p(i)-param.p_sp);
%     x_i = (T(i)-param.T_sp);
%     
%     J_inf = J_inf + x_i'*param.Q*x_i + ...
%                 u_i'*param.R*u_i;    
% end
% disp('J_inf: ');disp(J_inf);