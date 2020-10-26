clear; clc; close all;

FileName_robot = 'data_robot_SixBar_floating';
robot_data = load(FileName_robot);
robot = robot_data.robot;


r = robot.nodes_position;
result = get_linear_equations_statics('robot', robot, 'nodes_position', r, 'external_forces', zeros(size(r)));

P = eye(robot.number_of_nodes*3) - result.A * pinv(result.A);
error_0 = P*result.b;

%%%%%%%%%%%%%%%%%%%%%%%
% generate error matrix

delta = 0.001;

E = zeros(3*robot.number_of_nodes, 3*robot.number_of_nodes);

for i = 1:(3*robot.number_of_nodes)
    r = robot.nodes_position;
    r = r(:);
    r(i) = r(i) + delta;
    r = reshape(r, 3, []);
    
    result = get_linear_equations_statics('robot', robot, 'nodes_position', r, 'external_forces', zeros(size(r)));
    
    P = eye(robot.number_of_nodes*3) - result.A * pinv(result.A);
    E(:, i) = P*result.b;
end

rank(E)
size(E)
N = null(E);

%%%%%%%%%%%%%%%%%%%%%%%
% generate test that null space of error matrix indeed contais directions
% where the error does grow

dr = N*randn(size(N, 2), 1)*delta;
dr = N(:, 1)*delta;

r = robot.nodes_position + reshape(dr, 3, []);
result = get_linear_equations_statics('robot', robot, 'nodes_position', r, 'external_forces', zeros(size(r)));

P = eye(robot.number_of_nodes*3) - result.A * pinv(result.A);
errorN = P*result.b;

% compare with initia error, and typical errors
[error_0, errorN, E(:, 1:5)]

norm(error_0)
norm(errorN)
mean(vecnorm(E))

%%%%%%%%%%%%%%%%%%%%%%%%
% visualize
d = 0.2;

for i = 1:size(N, 2)
figure_handle = figure('Color', 'w');

vis_Draw(robot, robot.nodes_position, 'FaceAlpha', 0.10, ...
    'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
    'text_delta_x', 0.01, 'text_delta_z', 0.01, ...
    'CableFaceColor', [0 0.2 0], 'RodFaceColor',   [0.3 0.2 1]);

r = robot.nodes_position + reshape(N(:, i), 3, []) * d;
vis_Draw(robot, r, 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.03, 'RodsRadius', 0.01, 'CablesRadius', 0.002, ...
    'text_delta_x', 0.01, 'text_delta_z', 0.01, ...
    'CableFaceColor', [0 0.2 0.4], 'RodFaceColor',   [1 0.2 0.7]);

axis equal;
end

