function param = compute_controller_base_parameters
    % (1) load truck parameters
    load('system/parameters_truck');
    
    % (2) discretization
    Ts = 60;
    
    p = truck;
    A_c(1,1) = (-p.a12 - p.a1o)/p.m1;
    A_c(1,2) = p.a12/p.m1;
    A_c(1,3) = 0;
    A_c(2,1) = p.a12/p.m2;
    A_c(2,2) = (-p.a12-p.a23-p.a2o)/p.m2;
    A_c(2,3) = p.a23/p.m2;
    A_c(3,1) = 0;
    A_c(3,2) = p.a23/p.m3;
    A_c(3,3) = (-p.a23-p.a3o)/p.m3;

    B_c = [1/p.m1 0; 0 1/p.m2; 0 0];
    sys_continuous = ss(A_c, B_c, [1, 1, 1], 0);
    sys_discrete = c2d(sys_continuous, Ts);
    
    param.A = sys_discrete.A;
    param.B = sys_discrete.B;
    
    % (3) set point computation
    % Setting the disturbances 
    B_d = Ts*diag([1/p.m1, 1/p.m2,1/p.m3]);

    d = zeros(3,1);
    d(1) = p.a1o*p.To + p.w(1); 
    d(2) = p.a2o*p.To + p.w(2);
    d(3) = p.a3o*p.To + p.w(3);
    offset = B_d*d; 

    % (4) system constraints
    param.Pcons = truck.InputConstraints;
    param.Tcons = truck.StateConstraints;
    
    % (5) Compute_set_points as an optimization problem(param)
    [param.T_sp, param.p_sp] = ...
        compute_set_points(param, offset, [-20; 0.25]);
    param.Ucons = param.Pcons - param.p_sp;
    param.Xcons = param.Tcons - param.T_sp;
    
    % (6) LQR cost function
    param.Q = diag([500, 500, 0]);
    param.R = 0.01*eye(2);
   
end

function [xs, us] = compute_set_points(param, offset, reference)

    H = [1 0 0; 0 1 0];
    
    M_eq = [eye(3)-param.A, -param.B;
            H, zeros(2)];
    b_eq = [offset; reference];

    lb = [param.Tcons(:,1); param.Pcons(:,1)];
    ub = [param.Tcons(:,2); param.Pcons(:,2)];

    cost = zeros(5,5);
    cost(4:end,4:end) = eye(2);
    
    X = quadprog(cost,[],[],[], M_eq, b_eq, lb, ub, []);     
    xs = X(1:3);
    us = X(4:5);
end

% % ANALYTICAL CALCULATION OF SETPOINTS
%     T_sp = zeros(3,1);
%     T_sp(1) = -20;
%     T_sp(2) = 0.25;
%     T_sp(3) = 1/(1-A(3,3)) * (A(3,2)*T_sp(2) + offset(3));
%     
%     p_sp = zeros(2,1);
%     p_sp(1) = 1/B(1,1)*((1-A(1,1))*T_sp(1) -     A(1,2)*T_sp(2) - offset(1));
%     p_sp(2) = 1/B(2,2)*(   -A(2,1)*T_sp(1) + (1-A(2,2))*T_sp(2) - A(2,3)*T_sp(3) - offset(2));
% 
%     % Approximating the input set point surprisingly similar
%     p_sp = B\(T_sp - A*T_sp - offset);
