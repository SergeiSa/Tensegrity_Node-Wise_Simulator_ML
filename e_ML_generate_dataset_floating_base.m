%%% MAIN
clc; close all; clear;

Count = 20000;
range = 0.5;

rng('shuffle');

%%% Loading
% temp = load('data_robot_ThreePrizm_floating');
temp = load('data_robot_SixBar_floating');
robot = temp.robot;

%%% Setup
m = optimization_get_number_of_cables(robot);

initial_ro = robot.rest_lengths;

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);

cable_rest_lengths = zeros(Count, m); %X in the dataset
nodes_position = zeros(Count, robot.number_of_nodes*3); %Y in the dataset

x0 = robot.nodes_position;

%%% Generation
for i = 1:Count
    if rem(i, 100) == 0
        disp(['calculating ', num2str(i), ' out of ', num2str(Count)]);
    end

    diff = range*(rand(m, 1) - 0.5*ones(m, 1));
    
    new_ro = initial_ro + rho_handler.function_header(diff);
    
    x = solve_FK_fmincon_floatin_base(robot, new_ro, x0);

    cable_rest_lengths(i, :) = optimization_rho_vector_from_matrix(new_ro, rho_handler.map);
    nodes_position(i, :) = x(:);
end

save('C:\Sergei\NN datasets\2020 Tensegrity\data_ML_SixBar_FB_1', 'cable_rest_lengths', 'nodes_position');

