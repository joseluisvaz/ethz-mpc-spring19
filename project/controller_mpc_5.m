% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param yalmip_optimizer
persistent T_hat d_hat p_prev

global T_est
global d_est

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
    T_hat = T;
    d_hat = param.d;
end

%% compute state and disturbance estimate
% [T_hat, d_hat] = state_estimator(T, d, p_prev)

%% compute steady state x and u
H = diag([1,1,0]);
ss_vec = [param.A-eye(3) param.B; H*param.C zeros(3,2)]\...
         [-param.B_d*d_hat; param.r - H*param.C_d*d_hat];
T_s = ss_vec(1:3);
p_s = ss_vec(4:5);

%% evaluate control action by solving MPC problem, e.g.
x_hat = T_hat - T_s;
[u_mpc,errorcode] = yalmip_optimizer(x_hat);
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc + p_s;
% p_prev = p;
y = param.C*T;
est = param.A_aug*[T_hat;d_hat] + param.B_aug*p + ...
      param.L*(param.C*T_hat + param.C_d*d_hat - y);
T_hat = est(1:3);
T_est = [T_est T_hat];
d_hat = est(4:6);
d_est = [d_est d_hat];
end

% function [T_h, d_h] = state_estimator(T, d, p)
%     
%     y = param.C*T + param.C_d*d;
%     est = param.A_aug*[T_hat;d_hat] + param.B_aug*p + ...
%           param.L*(param.C*T_hat + param.C_d*d_hat - y);
% 
%     T_h = ;
%     d_h = ;
% end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and

param = compute_controller_base_parameters;

% Design observer
A_aug = [param.A param.B_d; zeros(3) eye(3)];
B_aug = [param.B; zeros(3,2)];
C_aug = [eye(3) zeros(3)];
L = -(place(A_aug',C_aug',[0,0,0,0.8,0.8,0.8]))';
param.A_aug = A_aug;
param.B_aug = B_aug;
param.C_aug = C_aug;
param.L = L;


% Yalmip optimizer object
[param.P, ~, ~] = dare(param.A, param.B, param.Q, param.R);

Ax_cons = [eye(3);-eye(3)];
bx_cons = [param.Xcons(:,2);-param.Xcons(:,1)];

Au_cons = [eye(2);-eye(2)];
bu_cons = [param.Ucons(:,2);-param.Ucons(:,1)];

N = 30;
nx = size(param.A,1);
nu = size(param.B,2);

U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');

objective = 0;
constraints = [];
for k = 1:N-1
  % system dynamic constraints
  constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
  % state constraints
  constraints = [constraints, Ax_cons*X{k+1} <= bx_cons];
  % input constraints
  constraints = [constraints, Au_cons*U{k} <= bu_cons];
  % objective, sum of stage cost function
  objective = objective + X{k}' * param.Q * X{k} + U{k}' * param.R * U{k};
end
l_f = X{end}'*param.P*X{end};
objective = objective + l_f;

x0 = sdpvar(3,1);
constraints = [constraints, X{1} == x0];

[A_X_LQR, b_X_LQR] = compute_X_LQR;
constraints = [constraints, A_X_LQR*X{N} <= b_X_LQR];


ops = sdpsettings('verbose',0,'solver','quadprog');
% fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = optimizer(constraints,objective,ops,x0,U{1});
end
