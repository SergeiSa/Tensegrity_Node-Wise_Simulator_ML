

% FileName_dataset = 'C:\Sergei\NN datasets\2020 Tensegrity\data_ML_3Prizm_FB_1';
% FileName_dataset = 'C:\Sergei\NN datasets\2020 Tensegrity\data_ML_SixBar_FB_1';
FileName_dataset = 'data_ML_SixBar_FB_1';

% FileName_robot = 'data_robot_ThreePrizm_floating';
FileName_robot = 'data_robot_SixBar_floating';

number_of_openings = 20;

opening_direction_1 = randn(1, 3);
opening_direction_2 = randn(1, 3);
slider = 0:(1/number_of_openings):1;
opening_direction = zeros(length(slider), 3);
for i = 1:length(slider)
    opening_direction(i, :) = slider(i)*opening_direction_1 + (1 - slider(i))*opening_direction_2;
end
% opening_width = 0.35*ones(length(slider), 1);
opening_width = 1.35*ones(length(slider), 1);

[result_cable_rest_lengths, result_node_positions, result_solution_found] = ...
    openings_get_best_fit_from_dataset('DatasetPath', FileName_dataset, ...
    'RobotPath', FileName_robot, ...
    'opening_direction', opening_direction, 'opening_width', opening_width);

%%%%%%%%%%%%%%%%%%%%%%%%%%

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);
i = 1;
r = reshape(result_node_positions(i, :), size(robot.nodes_position));
rho_matrix=optimization_rho_matrix_from_vector(result_cable_rest_lengths(i, :), rho_handler.map, robot);
    x = solve_FK_fmincon_floatin_base(robot, rho_matrix, r);

    vis_Draw(robot,x , 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
    'text_delta_x', 0.01, 'text_delta_z', 0.01);






robot_data = load(FileName_robot); 

figure_handle = figure('Color', 'w');

for i = 1:length(slider)
    
    A = [opening_direction(i, :); -opening_direction(i, :)];
    alpha = rand;
    b = [0.5*opening_width(i); 0.5*opening_width(i)];
    
    r = reshape(result_node_positions(i, :), 3, []);
    
    hold off;
    vis_Draw(robot_data.robot, r, 'FaceAlpha', 0.30, ...
        'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
        'text_delta_x', 0.01, 'text_delta_z', 0.01);
    
    vis_half_space(A(1, :), b(1), 'FaceColor', 'g'); hold on;
    vis_half_space(A(2, :), b(2), 'FaceColor', 'g');
    
    axis equal;
%     ax_scale = 2;
    ax_scale = 0.7;
    xlim([-1 1]*ax_scale);
    ylim([-1 1]*ax_scale);
    zlim([-1 1]*ax_scale);
    
    drawnow;
end