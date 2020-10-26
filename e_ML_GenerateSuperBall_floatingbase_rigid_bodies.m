clc; close all; clear;


robot.RobotName = 'SixBar_floating_rigid_bodies';


robot.number_of_nodes = 24;


Cables = zeros(robot.number_of_nodes, robot.number_of_nodes);

for i = 1:8
    Cables(i, i+8) = 1;
    Cables(i+8, i+16) = 1;
end

Cables(13, 14) = 1;
Cables(14, 15) = 1;
Cables(15, 16) = 1;
Cables(16, 13) = 1;
Cables(9,  10) = 1;
Cables(10, 11) = 1;
Cables(11, 12) = 1;
Cables(12, 9) = 1;

Cables = Cables + Cables';
if max(Cables(:)) > 1
    error('Something went wrong!')
end


Rods = zeros(robot.number_of_nodes, robot.number_of_nodes);

Rods(9,  13) = 1;
Rods(10, 14) = 1;
Rods(11, 15) = 1;
Rods(12, 16) = 1;

Rods = Rods + Rods';
if max(Rods(:)) > 1
    error('Something went wrong!')
end


robot.Connectivity = Cables + Rods;
robot.Cables = Cables;
robot.Rods = Rods;

robot.Bodies = {[9, 13], [10, 14], [11, 15], [12, 16], [1:8]};

robot.active_nodes = 1:robot.number_of_nodes;


L_cables = Cables * 0.15;
L_rods = Rods * 0.25;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 100;
mu_rods = Rods * 1000;
% mu_cables = Cables * 500;
% mu_rods = Rods * 2000;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_masses = ones(size(robot.Connectivity, 1), 1);

robot.nodes_position = [get_cube(zeros(3, 1), 0.5), ...
        diag([1;1;1.5])*get_cube(zeros(3, 1), 1.5), ...
                        get_cube(zeros(3, 1), 3.5)];

                    
robot.active_nodes = 1:16;                    
                   

% get_potential_energy_fnc_header = ...
%     get_potential_energy_fmincon_wrapper(robot.Connectivity, robot.nodes_position, ...
%                                          robot.stiffness_coef, robot.rest_lengths, robot.active_nodes);
% x = fminunc(get_potential_energy_fnc_header, robot.nodes_position(:, robot.active_nodes));
% 
% robot.nodes_position(:, robot.active_nodes) = x;


%%%%%%%%%%%%%%%%%%%%%

save(['data_robot_', robot.RobotName, '.mat'], 'robot')
%%%%%%%%%%%%%%%%%%%%%


figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position, 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.3, 'RodsRadius', 0.2, 'CablesRadius', 0.05, ...
    'text_delta_x', 0.01, 'text_delta_z', 0.01);

for i = 1:length(robot.Bodies)
    vis_Body( robot.nodes_position(:, robot.Bodies{i}) );
end

axis equal;

