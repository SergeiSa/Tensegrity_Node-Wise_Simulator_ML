clc; close all; clear;

Cables = zeros(2, 2);
Cables(1, 2) = 1;
Cables = math_add_symmetric_components(Cables);


Rods = zeros(2, 2);
Rods = math_add_symmetric_components(Rods);

robot.Connectivity = Cables + Rods;

L_cables = Cables * 0;
L_rods = Rods * 1;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 100;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_position = [    0.0   1
                            0.0   0
                            0.0   0];
robot.nodes_velocity = zeros(3, size(robot.Connectivity, 1));

% tests

r = sym('r', [3, 2]);
rho = sym('rho', [2, 2]);
k = sym('k', [2, 2]);

P = get_potential_energy(robot.Connectivity, r, k, rho)

% k1_2*(rho1_2 - (abs(r1_1 - r1_2)^2 + abs(r2_1 - r2_2)^2 + abs(r3_1 - r3_2)^2)^(1/2))^2 + 
% k1_3*(rho1_3 - (abs(r1_1 - r1_3)^2 + abs(r2_1 - r2_3)^2 + abs(r3_1 - r3_3)^2)^(1/2))^2 + 
% k1_4*(rho1_4 - (abs(r1_1 - r1_4)^2 + abs(r2_1 - r2_4)^2 + abs(r3_1 - r3_4)^2)^(1/2))^2 + 
% k2_1*(rho2_1 - (abs(r1_1 - r1_2)^2 + abs(r2_1 - r2_2)^2 + abs(r3_1 - r3_2)^2)^(1/2))^2 + 
% k1_6*(rho1_6 - (abs(r1_1 - r1_6)^2 + abs(r2_1 - r2_6)^2 + abs(r3_1 - r3_6)^2)^(1/2))^2 + 
% k2_3*(rho2_3 - (abs(r1_2 - r1_3)^2 + abs(r2_2 - r2_3)^2 + abs(r3_2 - r3_3)^2)^(1/2))^2 + 
% k2_4*(rho2_4 - (abs(r1_2 - r1_4)^2 + abs(r2_2 - r2_4)^2 + abs(r3_2 - r3_4)^2)^(1/2))^2 + 
% k2_5*(rho2_5 - (abs(r1_2 - r1_5)^2 + abs(r2_2 - r2_5)^2 + abs(r3_2 - r3_5)^2)^(1/2))^2 + 
% k3_1*(rho3_1 - (abs(r1_1 - r1_3)^2 + abs(r2_1 - r2_3)^2 + abs(r3_1 - r3_3)^2)^(1/2))^2 + 
% k3_2*(rho3_2 - (abs(r1_2 - r1_3)^2 + abs(r2_2 - r2_3)^2 + abs(r3_2 - r3_3)^2)^(1/2))^2 + 
% k3_5*(rho3_5 - (abs(r1_3 - r1_5)^2 + abs(r2_3 - r2_5)^2 + abs(r3_3 - r3_5)^2)^(1/2))^2 + 
% k4_1*(rho4_1 - (abs(r1_1 - r1_4)^2 + abs(r2_1 - r2_4)^2 + abs(r3_1 - r3_4)^2)^(1/2))^2 + 
% k3_6*(rho3_6 - (abs(r1_3 - r1_6)^2 + abs(r2_3 - r2_6)^2 + abs(r3_3 - r3_6)^2)^(1/2))^2 + 
% k4_2*(rho4_2 - (abs(r1_2 - r1_4)^2 + abs(r2_2 - r2_4)^2 + abs(r3_2 - r3_4)^2)^(1/2))^2 + 
% k5_2*(rho5_2 - (abs(r1_2 - r1_5)^2 + abs(r2_2 - r2_5)^2 + abs(r3_2 - r3_5)^2)^(1/2))^2 + 
% k5_3*(rho5_3 - (abs(r1_3 - r1_5)^2 + abs(r2_3 - r2_5)^2 + abs(r3_3 - r3_5)^2)^(1/2))^2 + 
% k6_1*(rho6_1 - (abs(r1_1 - r1_6)^2 + abs(r2_1 - r2_6)^2 + abs(r3_1 - r3_6)^2)^(1/2))^2 + 
% k6_3*(rho6_3 - (abs(r1_3 - r1_6)^2 + abs(r2_3 - r2_6)^2 + abs(r3_3 - r3_6)^2)^(1/2))^2



%%%%

f_array = get_elastic_force_sums_nodes_wrapper1(robot)

get_potential_energy_fnc_header = ...
    get_potential_energy_fmincon_wrapper(robot.Connectivity, robot.nodes_position, ...
                                         robot.stiffness_coef, robot.rest_lengths, [1; 2]);
x = fminunc(get_potential_energy_fnc_header, robot.nodes_position);

nodes_position = robot.nodes_position;
nodes_position = x;

f_array = get_elastic_force_sums_nodes(robot.Connectivity, nodes_position, ...
                                         robot.stiffness_coef, robot.rest_lengths)



