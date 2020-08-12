%function that finds the best fitting roboto position for a particular
%"opening" (space between two half spaces, indepndant of the tarnslations of the robot)
%
%Dataset - path to the dataset from which tje best fit is drawn
%Robot - path to the robot for which the best fit is drawn
%opening_direction - unit row vector (or a stack of them along 1st dimention) defining unaccessible half-spaces
%opening_width - scalar (or a stack of them along 1st dimention) defining distance between unaccessible half-spaces
%
function [result_cable_rest_lengths, result_node_positions, result_solution_found] = ...
    openings_get_best_fit_from_dataset(varargin)
Parser = inputParser;
Parser.FunctionName = 'openings_get_best_fit_from_dataset';
Parser.addOptional('DatasetPath', []);
Parser.addOptional('RobotPath', []);
Parser.addOptional('initial_rho', []);
Parser.addOptional('opening_direction', [0 0 1]);
Parser.addOptional('opening_width', 1);
Parser.parse(varargin{:});

dataset = load(Parser.Results.DatasetPath);
robot_data = load(Parser.Results.RobotPath);
robot = robot_data.robot;

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);

if isempty(Parser.Results.initial_rho)
    initial_rho = rho_handler.rho_vector_from_matrix(robot.rest_lengths)';
else
    initial_rho = reshape(Parser.Results.initial_rho, 1, []);
end

dataset_length = size(dataset.nodes_position, 1);
number_of_openings = size(Parser.Results.opening_direction, 1);

result_cable_rest_lengths = NaN(number_of_openings, size(dataset.cable_rest_lengths, 2));
result_node_positions     = NaN(number_of_openings, size(dataset.nodes_position, 2));
result_solution_found     = false(number_of_openings, 1);

for j = 1:number_of_openings
    
    %%% set up the opening
    opening_direction = Parser.Results.opening_direction(j, :);
    opening_direction = opening_direction / norm(opening_direction);
    
    opening_width = Parser.Results.opening_width(j);
    
    opening_compatibility = false(dataset_length, 1);
    
    for i = 1:dataset_length
        
        r = reshape(dataset.nodes_position(i, :), 3, []);
        z = opening_direction*r;
        
        z_range = minmax(z);
        if (z_range(2) - z_range(1)) < opening_width
            opening_compatibility(i) = true;
        end
        
    end
    
    if max(opening_compatibility) > 0
        compatible_rho = dataset.cable_rest_lengths(opening_compatibility, :);
        compatible_r   = dataset.nodes_position(opening_compatibility, :);
        
        diff_rho = compatible_rho - repmat(initial_rho, size(compatible_rho, 1), 1);
        diff_rho_norm = vecnorm(diff_rho');
        [~, min_index] = min(diff_rho_norm);
        
        result_cable_rest_lengths(j, :) = compatible_rho(min_index, :);
        result_node_positions(j, :)   = compatible_r(min_index, :);
        result_solution_found(j) = true;
    end
    
    
end

end