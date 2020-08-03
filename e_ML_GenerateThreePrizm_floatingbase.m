clc; close all; clear;

robot.RobotName = 'ThreePrizm_floating';

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

robot.active_nodes = 1:robot.number_of_nodes;

L_cables = Cables * 0.5;
L_rods = Rods * 3;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 100;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_masses = ones(size(robot.Connectivity, 1), 1);

robot.nodes_position = [    0.8660   -0.8660    0         1    -0.5    -0.5
                            0.5000    0.5000   -1.0000    0    -0.866   0.866
                            0         0         0         1    1        1];
                        
robot.z_regulirization_indices = [4, 5, 6];                      
                        
%%%%
% find stable IC

    
% %Cost design - beginning
% %get potential energy
% potential_energy_header = @(x) get_potential_energy(robot.Connectivity, reshape(x, 3, []), robot.stiffness_coef, robot.rest_lengths);
% 
% %regurilize for rotations
% reguliration_header = get_floating_base_Z_reguliration('indices', robot.z_regulirization_indices, 'nodes_position', robot.nodes_position);                                     
%         
% cost_weight_potential_eneggy = 1; 
% cost_weight_regulirization = 0.01;
% cost = @(x) cost_weight_potential_eneggy * potential_energy_header(x) + ...
%             cost_weight_regulirization   * reguliration_header(x);
% %Cost design - end
% 
% %Constraints design - beginning
% %constrain center of mass
% CoM_0 = get_CoM_v2(robot.nodes_masses, robot.nodes_position);        
% NONLCON = @(x) deal([], get_CoM_v2(robot.nodes_masses, x) - CoM_0);    
% %Constraints design - end                       
%                                      
% x = fmincon(cost, robot.nodes_position(:, robot.active_nodes), [],[],[],[],[],[], NONLCON);

x = solve_FK_fmincon_floatin_base(robot, robot.rest_lengths, robot.nodes_position);

robot.nodes_position(:, robot.active_nodes) = x;

save(['data_robot_', robot.RobotName, '.mat'], 'robot')

% f_array = get_elastic_force_sums_nodes(robot.Connectivity, nodes_position, ...
%                                          robot.stiffness_coef, robot.rest_lengths)
%
% result = optimization_generate_rho_vector_and_function(Cables);

%%%%

%%%%%%%%%%%%%%%%%%%%%%
%%%%% drawing 

p = robot.nodes_position;

figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position, 'FaceAlpha', 0.30);

axis equal;

