clear;
clc;
close all
%% Solve
% Declare the symbolic variables
syms mc mp b g T L F theta theta_dot theta_dot_dot x_dot_dot xp_dot_dot yp_dot_dot
% Define Equations
eq1 = mc*x_dot_dot == F + T*sin(theta);
eq2 = mp*xp_dot_dot == -T*sin(theta) + b*theta_dot*cos(theta);
eq3 = mp*yp_dot_dot == -mp*g + T*cos(theta) + b*theta_dot*sin(theta);
eq4 = xp_dot_dot == L*sin(theta)*theta_dot^2 - L*cos(theta)*theta_dot_dot + x_dot_dot;
eq5 = yp_dot_dot == -L*cos(theta)*theta_dot^2 - L*sin(theta)*theta_dot_dot;
% Solve - Force Domain
solutions_forcedomain = solve([eq1, eq2, eq3, eq4, eq5], [x_dot_dot, theta_dot_dot, xp_dot_dot, yp_dot_dot, T]);
display(solutions_forcedomain.x_dot_dot);
display(solutions_forcedomain.theta_dot_dot);
% Solve - Acceleration Domain
solutions_acceldomain = solve([eq1, eq2, eq3, eq4, eq5], [theta_dot_dot, xp_dot_dot, yp_dot_dot, T, F]);
display(solutions_acceldomain.theta_dot_dot);
% Acceleration Domain Modifiacation
display(solutions_acceldomain.theta_dot_dot);