% Init
clear all
close all
addpath(genpath(cd));
load('project/system/parameters_scenarios.mat');
load('project/system/parameters_truck.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
param = compute_controller_base_parameters;

% execute simulation starting from T0_1 using lqr controller with scenario 1
X0_1 = [3 1 0]';
X0_2 = [-1 -0.1 -4.5]';
T0_1 = param.T_sp + X0_1;
T0_2 = param.T_sp + X0_2;

% [T, p] = simulate_truck(T0_1, @controller_lqr,scen1);
% 
% % (5) Design LQR controller that converges reasonably fast
% if (norm(param.T_sp - T(:,31)) < 0.2*norm(X0))
%     disp("LQR satisfies convergence");
% else
%     disp("LQR does not satisfies convergence");
% end
% 
% % (6) Compute infinite horizon cost of LQR as a function of X0
% J_lqr = @(x, P) x'*param.P*x;
% 
% disp("LQR infinite horizon cost using dare: ");
% disp(J_lqr(X0, param.P));
% 
% % Lets compute the LQR with the lyapunov function
% A_cl = param.A + param.B*param.F;
% Q_cl = param.Q + param.F'*param.R*param.F;
% P_lyap = dlyap(A_cl, Q_cl);
% 
% disp("LQR infinite horizon cost using lyap equation: ");
% disp(J_lqr(X0, P_lyap));
% 
% % Sim system
% x_sim = zeros(3, 1001);
% x_sim(:,1) = X0; 
% for k = 1:1000
%     x_sim(:,k+1) = A_cl*x_sim(:,k);
% end
% 
% % Estimate infinite horizon cost
% J_est = 0;
% for k = 1:1001
%     J_est = J_est + x_sim(:,k)'*Q_cl*x_sim(:,k);
% end
% 
% disp("Estimate LQR infinite horizon numerically");
% disp(J_est);
% 
% compute_X_LQR();

% figure(1)

global estimate_s
estimate_s = [T0_1; param.d];
[T1, p1] = simulate_truck(T0_1, @controller_mpc_5, scen3);
% figure(2)
% [T2, p2] = simulate_truck(T0_2, @controller_mpc_4, scen1);

figure(2) 
subplot(6,1,1)
plot(T1(1,:)); hold on;
plot(estimate_s(1,:), '--r');
subplot(6,1,2)
plot(T1(2,:)); hold on;
plot(estimate_s(2,:), '--r');
subplot(6,1,3)
plot(T1(3,:)); hold on;
plot(estimate_s(3,:), '--r');
subplot(6,1,4)
plot(estimate_s(4,:), '--r');
subplot(6,1,5)
plot(estimate_s(5,:), '--r');
subplot(6,1,6)
plot(estimate_s(6,:), '--r');

