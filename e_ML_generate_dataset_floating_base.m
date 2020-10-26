%%% MAIN
clc; close all; clear;

Count = 5;
range = 0.5;

rng('shuffle');

%%% Loading
% temp = load('data_robot_ThreePrizm_floating');
temp = load('data_robot_SixBar_floating');
robot = temp.robot;

%%% Setup
m = optimization_get_number_of_cables(robot);

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);

initial_ro___cables_and_rods = robot.rest_lengths;
initial_ro_vector___cables = rho_handler.rho_vector_from_matrix(initial_ro___cables_and_rods);
initial_ro_matrix___cables = rho_handler.rho_matrix_from_vector(initial_ro_vector___cables);


cable_rest_lengths = zeros(Count, m); %X in the dataset
nodes_position = zeros(Count, robot.number_of_nodes*3); %Y in the dataset

x0 = robot.nodes_position;

%%% Generation
for i = 1:Count
    if rem(i, 100) == 0
        disp(['calculating ', num2str(i), ' out of ', num2str(Count)]);
    end

    diff = range*(rand(m, 1) - 0.5*ones(m, 1));
    
    peturbed_rho___cables_and_rods = initial_ro___cables_and_rods + rho_handler.rho_matrix_from_vector(diff);
    
    x = solve_FK_fmincon_floatin_base(robot, peturbed_rho___cables_and_rods, x0);

%     close all;
%         vis_Draw(robot,x , 'FaceAlpha', 0.30, ...
%     'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
%     'text_delta_x', 0.01, 'text_delta_z', 0.01);
% axis equal;    

    %cable_rest_lengths(i, :) = rho_handler.rho_vector_from_matrix(new_ro, rho_handler.map);
    cable_rest_lengths(i, :) = initial_ro_vector___cables + diff;
    nodes_position(i, :) = x(:);
    
end

%%

rho_handler_Cables = optimization_generate_rho_vector_and_function(robot.Cables);
rho_handler_Rods = optimization_generate_rho_vector_and_function(robot.Rods);

i = 1;

rho_cables = rho_handler_Cables.rho_matrix_from_vector(cable_rest_lengths(i, :)');
% rho_rods = rho_handler_Cables.rho_matrix_from_vector(cable_rest_lengths(i, :)');

rho_rods = robot.rest_lengths;
rho_rods(logical(robot.Cables)) = 0;


rho_matrix=rho_cables+rho_rods;
    x = solve_FK_fmincon_floatin_base(robot, rho_matrix, x0);

    clf;
    vis_Draw(robot,x , 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
    'text_delta_x', 0.01, 'text_delta_z', 0.01);
axis equal



% save('data_ML_SixBar_FB_1', 'cable_rest_lengths', 'nodes_position');
% save('C:\Sergei\NN datasets\2020 Tensegrity\data_ML_SixBar_FB_1', 'cable_rest_lengths', 'nodes_position');

