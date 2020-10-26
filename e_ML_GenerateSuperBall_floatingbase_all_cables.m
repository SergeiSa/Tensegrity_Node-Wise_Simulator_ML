clc; close all; clear;


robot.RobotName = 'SixBar_floating_all_cables';


robot.number_of_nodes = 12;


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


Cables = ones(robot.number_of_nodes, robot.number_of_nodes);
% Cables = Cables - Rods;
Cables(logical(Rods)) = 0;
Cables(logical(eye(robot.number_of_nodes))) = 0;


robot.Connectivity = Cables + Rods;
robot.Cables = Cables;
robot.Rods = Rods;

robot.active_nodes = 1:robot.number_of_nodes;


robot.rest_lengths_cables = Cables * 0.25;
robot.rest_lengths_rods = Rods * 0.5;
robot.rest_lengths = robot.rest_lengths_cables + robot.rest_lengths_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 100;
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



%%%%%%%%%%%%%%%%%%%%%%%%%%
result = get_linear_equations_statics('robot', robot, 'nodes_position', robot.nodes_position);

N = null(result.A);

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);



get_elastic_force_sums_nodes(robot.Connectivity, robot.nodes_position, robot.stiffness_coef, robot.rest_lengths)


delta = 0.001;
delta_rho = N*randn(size(N, 2), 1)*delta;
rho_peterbed = rho_handler.rho_matrix_from_vector (rho_handler.rho_vector_from_matrix(robot.rest_lengths_cables) + delta_rho);
rest_lengths_peterbed = rho_peterbed + robot.rest_lengths_rods;

get_elastic_force_sums_nodes(robot.Connectivity, robot.nodes_position, robot.stiffness_coef, rest_lengths_peterbed)

%rho_handler.rho_matrix_from_vector
%rho_handler.rho_vector_from_matrix




