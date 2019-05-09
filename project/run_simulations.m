% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
param = compute_controller_base_parameters;
% execute simulation starting from T0_1 using lqr controller with scenario 1
x0_1 = [3 1 0]';
T0_1 = param.T_sp + x0_1;

x0_2 = [-1 -0.1 -4.5]';
T0_2 = param.T_sp + x0_2;
[T, p] = simulate_truck(T0_2, @controller_mpc_3, scen2);

err = norm(param.T_sp-T(:,30));
if err <= 0.2*norm(x0_2)
    disp('Performance achieved');
else
    disp('Not good enough');
end

%%
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