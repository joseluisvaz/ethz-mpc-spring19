function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    
%     (2) discretization
    Ts = 60;
    t = truck;
    A = zeros(3,3);
    A(1,1) = 1 + Ts/t.m1*(-t.a12-t.a1o);
    A(1,2) = Ts/t.m1*t.a12;
    A(1,3) = 0;
    A(2,1) = Ts/t.m2*t.a12;
    A(2,2) = 1 + Ts/t.m2*(-t.a12-t.a23-t.a2o);
    A(2,3) = Ts/t.m2*t.a23;
    A(3,1) = 0;
    A(3,2) = Ts/t.m3*t.a23;
    A(3,3) = 1 + Ts/t.m3*(-t.a23-t.a3o);
    B = [Ts/t.m1 0;...
         0 Ts/t.m2;...
         0 0];

    B_d = [Ts/t.m1 0 0;...
           0 Ts/t.m2 0;...
           0 0 Ts/t.m3];
    
    d = [t.a1o*t.To + t.w(1);...
         t.a2o*t.To + t.w(2);...
         t.a3o*t.To + t.w(3)];
    
    % (3) set point computation
    T_1s = -20;
    T_2s = 0.25;
%     T_3s = A(3,2)/(1-A(3,3))*0.25 + 60/t.m3/(1-A(3,3))*d(3);
    T_3s = 1/(1-A(3,3))*(A(3,1)*T_1s + A(3,2)*T_2s + ...
            B_d(3,1)*d(1) + B_d(3,2)*d(2) + B_d(3,3)*d(3));
    
    T_sp = [T_1s;T_2s;T_3s];
    p_sp = B\(T_sp - A*T_sp - B_d*d);

    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = [Pcons(:,1)-p_sp, Pcons(:,2)-p_sp];
    Xcons = [Tcons(:,1)-T_sp, Tcons(:,2)-T_sp];
    
    % (5) LQR cost function
    Q = diag([500, 1000, 0]);
    R = diag([0.01, 0.005]);
    
    % put everything together
    param.A = A;
    param.B = B;
    param.B_d = B_d;
    param.C = eye(3);
    param.C_d = zeros(3);
    param.d = d;
    param.Q = Q;
    param.R = R;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
    param.r = [T_1s;T_2s;0];
   
end

