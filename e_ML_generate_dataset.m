%%% MAIN
clc; close all; clear;

Count = 100000;
range = 0.1;

rng('shuffle');

%%% Loading
temp = load('data_robot_ThreePrizm');
robot = temp.robot;

%%% Setup
m = optimization_get_number_of_cables(robot);

initial_ro = robot.rest_lengths;

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);

cable_rest_lengths = zeros(Count, m);
nodes_position = zeros(Count, length(robot.active_nodes)*3);

options = optimoptions('fminunc', 'Display', 'none');

%%% Generation
for i = 1:Count
    if rem(i, 100) == 0
        disp(['calculating ', num2str(i), ' out of ', num2str(Count)]);
    end

    diff = range*(rand(m, 1) - 0.5*ones(m, 1));
    
    new_ro = initial_ro + rho_handler.function_header(diff);
    
    get_potential_energy_fnc_header = ...
    get_potential_energy_fmincon_wrapper(robot.Connectivity, robot.nodes_position, ...
                                         robot.stiffness_coef, new_ro, robot.active_nodes);
                                     
    x = fminunc(get_potential_energy_fnc_header, robot.nodes_position(:, robot.active_nodes), options);

    cable_rest_lengths(i, :) = optimization_rho_vector_from_matrix(new_ro, rho_handler.map);
    nodes_position(i, :) = x(:);
end

save('data_ML_3Prizm_1', 'cable_rest_lengths', 'nodes_position');

