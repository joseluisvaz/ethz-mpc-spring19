function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    
    % (2) discretization
    Ts = 60;

    % Parameter vector p
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
    A = sys_discrete.A;
    B = sys_discrete.B;
    
    % (3) set point computation
    % Setting the disturbances 
    B_d = diag([1/p.m1, 1/p.m2,1/p.m3]);

    d = zeros(3,1);
    d(1) = p.a1o*p.To + p.w(1); 
    d(2) = p.a2o*p.To + p.w(2);
    d(3) = p.a3o*p.To + p.w(3);
    offset = B_d*d'; 

    % Is T_sp(3) calculated correctly
    T_sp = zeros(3,1);
    T_sp(1) = -20;
    T_sp(2) = 0.25;
    T_sp(3) = 1/(1-A(3,3)) * (A(3,2)*T_sp(2) + offset(3));
    
    p_sp = zeros(2,1);
    p_sp(1) = 1/B(1,1)*((1-A(1,1))*T_sp(1) -     A(1,2)*T_sp(2) - offset(1));
    p_sp(2) = 1/B(2,2)*(   -A(2,1)*T_sp(1) + (1-A(2,2))*T_sp(2) - A(2,3)*T_sp(3) - offset(2));

    % Approximating the input set point surprisingly similar
    prov = B\(T_sp - A*T_sp - offset);
    
    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons - p_sp; 
    Xcons = Tcons - T_sp; 
    
    % (5) LQR cost function
    Q = eye(3); 
    R = eye(2); 
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end

