clc; close all; clear;


robot.RobotName = 'SixBar_floating';


robot.number_of_nodes = 12;


Cables = zeros(robot.number_of_nodes, robot.number_of_nodes);

Cables(1, 5) = 1;
Cables(1, 6) = 1;
Cables(1, 9) = 1;
Cables(1, 11) = 1;

Cables(3, 5) = 1;
Cables(3, 6) = 1;
Cables(3, 10) = 1;
Cables(3, 12) = 1;


Cables(2, 7) = 1;
Cables(2, 8) = 1;
Cables(2, 9) = 1;
Cables(2, 11) = 1;

Cables(4, 7) = 1;
Cables(4, 8) = 1;
Cables(4, 10) = 1;
Cables(4, 12) = 1;


Cables(7, 9) = 1;
Cables(7, 10) = 1;

Cables(5, 9) = 1;
Cables(5, 10) = 1;

Cables(8, 11) = 1;
Cables(8, 12) = 1;

Cables(6, 11) = 1;
Cables(6, 12) = 1;




Cables = Cables + Cables';
if max(Cables(:)) > 1
    error('Something went wrong!')
end


Rods = zeros(robot.number_of_nodes, robot.number_of_nodes);
Rods(1, 2) = 1;
Rods(3, 4) = 1;
Rods(5, 6) = 1;
Rods(7, 8) = 1;
Rods(9, 10) = 1;
Rods(11, 12) = 1;


Rods = Rods + Rods';
if max(Rods(:)) > 1
    error('Something went wrong!')
end


robot.Connectivity = Cables + Rods;
robot.Cables = Cables;
robot.Rods = Rods;

robot.active_nodes = 1:robot.number_of_nodes;


L_cables = Cables * 0.25;
L_rods = Rods * 0.5;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 1000;
% mu_cables = Cables * 500;
% mu_rods = Rods * 2000;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_masses = ones(size(robot.Connectivity, 1), 1);


nodes_position1 = [-0.5 -0.5  0.5 0.5;
                    0  0  0  0;
                    -1 1 -1  1];
                
nodes_position2 = [ 0  0  0  0;
                   -1 1 -1  1;
                   -0.5 -0.5  0.5 0.5];
                
nodes_position3 = [-1 1 -1  1;
                   -0.5 -0.5  0.5 0.5;
                    0  0  0  0];                
                
robot.nodes_position  = [nodes_position1, nodes_position2, nodes_position3]*0.5;               

robot.z_regularization_indices = [9, 10, 11, 12];   

%%%%%%%%%%%%%%%%%%%%%
x = solve_FK_fmincon_floatin_base(robot, robot.rest_lengths, robot.nodes_position);

robot.nodes_position(:, robot.active_nodes) = x;

save(['data_robot_', robot.RobotName, '.mat'], 'robot')
%%%%%%%%%%%%%%%%%%%%%


figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position, 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
    'text_delta_x', 0.01, 'text_delta_z', 0.01);

axis equal;


%%%%%%%%%%%%%%%%%%%%%
%IK demonstration
    

initial_ro = robot.rest_lengths;
desired_nodes_position = robot.nodes_position;

active_nodes = [1, 3];

desired_nodes_position(:, active_nodes(1)) = desired_nodes_position(:, active_nodes(1)) + [0.15; 0.15; 0];
desired_nodes_position(:, active_nodes(2)) = desired_nodes_position(:, active_nodes(2)) + [-0.15; -0.15; 0];

% [r, rho] = solve_IK_fmincon_closest_pose_for_given_one(robot, desired_nodes_position, initial_ro, ...
%     'options', optimoptions('fmincon', 'Display', 'iter', 'MaxFunctionEvaluations', 30000));

[r, rho] = solve_IK_fmincon(robot, desired_nodes_position, initial_ro, ...
    'active_nodes', active_nodes, ...
    'options', optimoptions('fminunc', 'Display', 'iter', 'MaxFunctionEvaluations', 300000));


figure_handle = figure('Color', 'w');
vis_Draw(robot, r, 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
    'text_delta_x', 0.01, 'text_delta_z', 0.01);

plot3(desired_nodes_position(1, active_nodes(1)), ...
    desired_nodes_position(2, active_nodes(1)), ...
    desired_nodes_position(3, active_nodes(1)), ...
    'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(r(1, active_nodes(1)), r(2, active_nodes(1)), r(3, active_nodes(1)), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

plot3(desired_nodes_position(1, active_nodes(2)), ...
    desired_nodes_position(2, active_nodes(2)), ...
    desired_nodes_position(3, active_nodes(2)), ...
    'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(r(1, active_nodes(2)), r(2, active_nodes(2)), r(3, active_nodes(2)), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

axis equal;