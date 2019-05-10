% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_4(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
x = T - param.T_sp;
[u_mpc,errorcode] = yalmip_optimizer(x);
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters
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
