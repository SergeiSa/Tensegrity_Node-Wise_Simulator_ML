clc; close all; clear;

robot.RobotName = 'ThreePrizm_floating_bodies';

robot.number_of_nodes = 6;

Cables = zeros(robot.number_of_nodes, robot.number_of_nodes);
Cables(1, 2) = 1;
Cables(1, 3) = 1;
Cables(2, 3) = 1;

Cables(1, 4) = 1;
Cables(2, 5) = 1;
Cables(3, 6) = 1;

Cables(4, 5) = 1;
Cables(4, 6) = 1;
Cables(5, 6) = 1;

Cables = Cables + Cables';
if max(Cables(:)) > 1
    error('Something went wrong!')
end


Rods = zeros(robot.number_of_nodes, robot.number_of_nodes);
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

robot.Bodies = {[1, 6], [2, 4], [3, 5]};

robot.active_nodes = 1:robot.number_of_nodes;

L_cables = Cables * 0.5;
L_rods = Rods * 3;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 1000;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_masses = ones(size(robot.Connectivity, 1), 1);

robot.nodes_position = [    0.8660   -0.8660    0         1    -0.5    -0.5
                            0.5000    0.5000   -1.0000    0    -0.866   0.866
                            0         0         0         1    1        1];
                        
robot.z_regularization_indices = [4, 5, 6];                      
                        
%%%%
% find stable IC

x = solve_FK_fmincon_floatin_base(robot, robot.rest_lengths, robot.nodes_position);

robot.nodes_position(:, robot.active_nodes) = x;

save(['data_robot_', robot.RobotName, '.mat'], 'robot')

robot.Rods = [];
robot.stiffness_coef = mu_cables;
robot.Connectivity = Cables;

%%%%

%%%%%%%%%%%%%%%%%%%%%%
%%%%% drawing 

figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position, 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.1, 'RodsRadius', 0.05, 'CablesRadius', 0.02, ...
    'text_delta_x', 0.05, 'text_delta_z', 0.05);

for i = 1:length(robot.Bodies)
    vis_Body( robot.nodes_position(:, robot.Bodies{i}), 'PaddingRadius', 0.05, 'EdgeAlpha', 0.1);
end

axis equal;

