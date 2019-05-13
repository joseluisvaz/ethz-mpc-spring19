% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    
    % Lets compute the LQR controller
    
    K = -dlqr(param.A, ...
              param.B, ...
              param.Q, ...
              param.R);
    
    % Define Autonomous System under LQR Feeback
    systemLQR = LTISystem('A', param.A + param.B*K);
    
    % Express the constraints as a Polhedron Xp  (Ax <= b)
    Xcons = param.Xcons;
    Ucons = param.Ucons;
    
    A = [eye(3); -eye(3); K; -K];
    b = [Xcons(:, 2); -Xcons(:, 1); Ucons(:, 2); -Ucons(:, 1)];     
    Xp = Polyhedron('A', A, 'b', b);
        
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    
    InvSetLQR = systemLQR.invariantSet();
%     figure(1)
%     InvSetLQR.plot(), alpha(0.25)
    
    %% Here you need to implement the X_LQR computation and assign the result.
    A_x = InvSetLQR.A;
    b_x = InvSetLQR.b;
end
