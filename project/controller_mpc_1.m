% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_1(T)

% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
% [u_mpc,errorcode] = yalmip_optimizer(...);
% if (errorcode ~= 0)
%       warning('MPC infeasible');
% end
% p = ...;

X = T - param.T_sp;

[u_mpc,errorcode] = yalmip_optimizer(X);
if (errorcode ~= 0)
      warning('MPC infeasible');
end

p = u_mpc + param.p_sp;


end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

% get basic controller parameters
param = compute_controller_base_parameters; 

%% implement your MPC using Yalmip here, e.g.
N = 30;
nx = size(param.A,1);
nu = size(param.B,2);

U = sdpvar(repmat(nu,1,N-1), repmat(1,1,N-1),'full');
X = sdpvar(repmat(nx,1,N), repmat(1,1,N),'full');

objective = 0;
constraints = [];

% Constraints for initial stage, no state constraints
constraints = [constraints, ...
  X{2} == param.A*X{1} + param.B*U{1}];
constraints = [constraints, ...
  param.Ucons(:,1) <= U{1} <= param.Ucons(:,2)];
objective = objective + X{1}' * param.Q * X{1} ...
                    + U{1}' * param.R * U{1};
                    
for k = 2:N-1
  % Equality constraints dynamics
  constraints = [constraints, ...
      X{k+1} == param.A*X{k} + param.B*U{k}];
  constraints = [constraints, ...
      param.Xcons(:,1) <= X{k} <= param.Xcons(:,2)];
  constraints = [constraints, ...
      param.Ucons(:,1) <= U{k} <= param.Ucons(:,2)];
  
  objective = objective + X{k}' * param.Q * X{k} ...
                        + U{k}' * param.R * U{k};
end

% Terminal Objective
objective = objective +  X{end}' * param.P * X{end};

% parameter for initial condition
x0 = sdpvar(3,1);
constraints = [constraints, X{1} == x0];

ops = sdpsettings('verbose',0,'solver','quadprog');
% fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = optimizer(constraints,objective,ops, x0, U{1});
end