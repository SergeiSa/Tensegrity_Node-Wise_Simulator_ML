clear; clc; close all;

% FileName_dataset = 'C:\Sergei\NN datasets\2020 Tensegrity\data_ML_3Prizm_FB_1';
FileName_dataset = 'C:\Sergei\NN datasets\2020 Tensegrity\data_ML_SixBar_FB_1';
dataset = load(FileName_dataset);

% FileName_robot = 'data_robot_ThreePrizm_floating';
FileName_robot = 'data_robot_SixBar_floating';
robot_data = load(FileName_robot); 
robot = robot_data.robot;

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);
initial_ro = rho_handler.rho_vector_from_matrix(robot.rest_lengths)';

dataset_length = size(dataset.nodes_position, 1);

%%% set up the opening
opening_direction = randn(1, 3);
opening_direction = opening_direction / norm(opening_direction);

opening_width = 0.25;

opening_compatibility = false(dataset_length, 1);
temp_range = zeros(dataset_length, 1);

for i = 1:dataset_length
    
r = reshape(dataset.nodes_position(i, :), 3, []);
z = opening_direction*r;

z_range = minmax(z);
if (z_range(2) - z_range(1)) < opening_width
    opening_compatibility(i) = true;
end

temp_range(i) = z_range(2) - z_range(1);
end

minmax(opening_compatibility')
minmax(temp_range')

if max(opening_compatibility) > 0
    compatible_rho = dataset.cable_rest_lengths(opening_compatibility, :);
    compatible_r   = dataset.nodes_position(opening_compatibility, :);
    
    diff_rho = compatible_rho - repmat(initial_ro, size(compatible_rho, 1), 1);
    diff_rho_norm = vecnorm(diff_rho');
    [~, min_index] = min(diff_rho_norm);
    best_rho = compatible_rho(min_index, :);
    best_rho
    
    A = [opening_direction; -opening_direction];
    alpha = rand;
    b = [alpha*opening_width; (1-alpha)*opening_width] * 1.01;
    
    r = reshape(compatible_r(min_index, :), 3, []);
    [new_nodes_position, shift] = fit_robot_to_area(r, A, b);
    shift
    
    figure_handle = figure('Color', 'w');
    vis_Draw(robot, new_nodes_position, 'FaceAlpha', 0.30, ...
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
end



% temp.cable_rest_lengths
% temp.nodes_position