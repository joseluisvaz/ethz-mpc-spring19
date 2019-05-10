param = compute_controller_base_parameters;
load('system/parameters_truck');
t = truck;
Ts = 60;
Bd = Ts*diag([1/t.m1, 1/t.m2, 1/t.m3]);
A_aug = [param.A Bd; zeros(3) eye(3)];
B_aug = [param.B; zeros(3,2)];
C_aug = [eye(3) zeros(3)];
D_aug = zeros(3,2);

L = -(place(A_aug',C_aug',[0,0,0,0.5,0.5,0.5]))';
eig(A_aug + L*C_aug)
