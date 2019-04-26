% Init
clear all
close all
addpath(genpath(cd));
load('project/system/parameters_scenarios.mat');
load('project/system/parameters_truck.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 

% execute simulation starting from T0_1 using lqr controller with scenario 1
X0 = [3 1 0]';
T0_1 = T_sp + X0;
[T, p] = simulate_truck(T0_1, @controller_lqr,scen1);
