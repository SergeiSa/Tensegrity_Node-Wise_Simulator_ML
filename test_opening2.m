

% FileName_dataset = 'C:\Sergei\NN datasets\2020 Tensegrity\data_ML_3Prizm_FB_1';
FileName_dataset = 'C:\Sergei\NN datasets\2020 Tensegrity\data_ML_SixBar_FB_1';

% FileName_robot = 'data_robot_ThreePrizm_floating';
FileName_robot = 'data_robot_SixBar_floating';

number_of_openings = 20;

opening_direction = randn(number_of_openings, 3);
opening_width = rand(number_of_openings, 1) + 0.05;


[result_cable_rest_lengths, result_node_positions, result_solution_found] = ...
    openings_get_best_fit_from_dataset('DatasetPath', FileName_dataset, ...
    'RobotPath', FileName_robot, ...
    'opening_direction', opening_direction, 'opening_width', opening_width);

%%%%%%%%%%
% dataset collection

result_cable_rest_lengths = result_cable_rest_lengths(result_solution_found, :);
result_node_positions     = result_node_positions(result_solution_found, :);

opening_direction = opening_direction(result_solution_found, :);
opening_width = opening_width(result_solution_found);

Y = result_cable_rest_lengths;
X = [opening_direction, opening_width];