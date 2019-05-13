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
persistent estimate A_aug B_aug C_aug observer_gain
persistent setPointSelector

global estimate_s

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
    estimate = [T; param.d];
    
    % Design the Observer
    A_aug = [param.A, param.B_d; zeros(3), eye(3)];
    B_aug = [param.B; zeros(3,2)];
    C_aug = [eye(3), zeros(3)];
    observer_gain = -(place(A_aug', C_aug', [0 0 0 0.3 0.3 0.3]))';
    
    % Selection matrix
    H = [1 0 0; 0 1 0];
    A_selector = [param.A - eye(3), param.B; H*eye(3), zeros(2,2)];
    b_selector = @(d) [-param.B_d*d; param.r];
    setPointSelector = @(d) A_selector\b_selector(d);
end

set_point = setPointSelector(estimate(4:end));

X_estimate = estimate(1:3) - set_point(1:3);

% Online set point
ts = set_point(1:3);
ps = set_point(4:end);

[u_mpc,errorcode] = yalmip_optimizer([X_estimate; ts; ps]);
if (errorcode ~= 0)
      warning('MPC infeasible');
end

p = u_mpc + set_point(4:end);

% Update the observer estimate
estimate = A_aug*estimate + B_aug*p ...
    + observer_gain*(-T + C_aug*estimate);

estimate_s = [estimate_s, estimate];

end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters
[param.A_x, param.b_x] = compute_X_LQR;

%% implement your MPC using Yalmip here, e.g.
N = 30;
nx = size(param.A,1);
nu = size(param.B,2);

% Slack costs
linear_cost_slack = 20000;
quadratic_cost_slack = 2000*eye(nx);

U = sdpvar(repmat(nu,1,N-1), repmat(1,1,N-1),'full');
X = sdpvar(repmat(nx,1,N), repmat(1,1,N),'full');
S = sdpvar(repmat(nx,1,N), repmat(1,1,N),'full');

objective = 0;
constraints = [];

% Parameters for constraints
ts = sdpvar(3,1);
ps = sdpvar(2,1);


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
  % Inequality constraints
  constraints = [constraints, ...
      param.Tcons(:,1) - ts - S{k} <= X{k} <= ...
      param.Tcons(:,2) - ts + S{k}];
  constraints = [constraints, ...
      param.Pcons(:,1) - ps <= U{k} <= param.Ucons(:,2) - ps];
  % Constraints on slack
  constraints = [constraints, S{k} >= 0];
  
  % Nominal cost
  objective = objective + X{k}' * param.Q * X{k} ...
                        + U{k}' * param.R * U{k};
  
  % Slack cost
  objective = objective + linear_cost_slack*norm(S{k},inf) ...
                        + S{k}' * quadratic_cost_slack * S{k};
end

% Terminal Objective
objective = objective +  X{end}' * param.P * X{end};

% Terminal Slack cost
objective = objective + linear_cost_slack*norm(S{end},inf) ...
                      + S{end}' * quadratic_cost_slack * S{end};

% Terminal Constraints (LQR InvSet)
constraints = [constraints, ...
    param.A_x * X{end} <= param.b_x];

% parameter for initial condition
x0 = sdpvar(3,1);
constraints = [constraints, X{1} == x0];

ops = sdpsettings('verbose',0,'solver','quadprog');
% fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = ...
    optimizer(constraints, objective, ops, [x0; ts; ps], U{1});

end