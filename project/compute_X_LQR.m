% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    
    K = -dlqr(param.A,param.B,param.Q,param.R);
    systemLQR = LTISystem('A', param.A + param.B*K);
    
    Acons = [eye(3);-eye(3);K;-K];
    bcons = [param.Xcons(:,2);-param.Xcons(:,1);...
             param.Ucons(:,2);-param.Ucons(:,1)];
    Xp = Polyhedron('A',Acons, 'b', bcons);
    
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    
    InvSetLQR = systemLQR.invariantSet();
    figure(1)
    hold on
    InvSetLQR.plot(), alpha(0.5), xlabel('x_1'), ylabel('x_2'), zlabel('x_3');
    
    A_x = InvSetLQR.A;
    b_x = InvSetLQR.b;
end

