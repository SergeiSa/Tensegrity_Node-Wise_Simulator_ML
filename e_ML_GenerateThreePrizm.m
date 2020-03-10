clc; close all; clear;

robot.RobotName = 'ThreePrizm';

Cables = zeros(6, 6);
Cables(1, 2) = 1;
Cables(1, 3) = 1;
Cables(1, 4) = 1;
Cables(2, 3) = 1;
Cables(2, 5) = 1;
Cables(3, 6) = 1;

Cables = Cables + Cables';
if max(Cables(:)) > 1
    error('Something went wrong!')
end


Rods = zeros(6, 6);
Rods(1, 6) = 1;
Rods(2, 4) = 1;
Rods(3, 5) = 1;

Rods = Rods + Rods';
if max(Rods(:)) > 1
    error('Something went wrong!')
end

robot.Connectivity = Cables + Rods;
robot.Cables = Cables;
robot.Rods = Rods;

robot.active_nodes = [1; 2; 3];

L_cables = Cables * 0.5;
L_rods = Rods * 2;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 100;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_position = [    0.8660   -0.8660    0         0.6411   -0.2979   -0.3139
                            0.5000    0.5000   -1.0000    0.0154    0.5613   -0.5585
                            0         0         0         1.0365    1.0277    1.0347];
%%%%
% find stable IC


% f_array = get_elastic_force_sums_nodes_wrapper1(robot)

get_potential_energy_fnc_header = ...
    get_potential_energy_fmincon_wrapper(robot.Connectivity, robot.nodes_position, ...
                                         robot.stiffness_coef, robot.rest_lengths, robot.active_nodes);
x = fminunc(get_potential_energy_fnc_header, robot.nodes_position(:, robot.active_nodes));

robot.nodes_position(:, robot.active_nodes) = x;

save(['data_robot_', robot.RobotName, '.mat'], 'robot')

% f_array = get_elastic_force_sums_nodes(robot.Connectivity, nodes_position, ...
%                                          robot.stiffness_coef, robot.rest_lengths)
%
% result = optimization_generate_rho_vector_and_function(Cables);

%%%%


