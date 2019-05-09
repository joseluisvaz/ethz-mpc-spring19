% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

% compute control action
u = param.F*(T - param.T_sp);
p = u + param.p_sp;

end

function param = init()
param = compute_controller_base_parameters;
% add additional parameters if necessary, e.g.
[~, ~, param.F] = dare(param.A, param.B, param.Q, param.R);
param.F = -param.F;

end