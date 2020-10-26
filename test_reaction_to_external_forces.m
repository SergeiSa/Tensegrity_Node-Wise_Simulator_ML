clear; clc; close all;

% FileName_robot = 'data_robot_SixBar_floating';
FileName_robot = 'data_robot_SixBar_floating_all_cables';
robot_data = load(FileName_robot);
robot = robot_data.robot;


r = robot.nodes_position;
result = get_linear_equations_statics('robot', robot, 'nodes_position', r, 'external_forces', zeros(size(r)));

I = eye(robot.number_of_nodes*3);
P = I - result.A*pinv(result.A);

error = P * (repmat(result.b, 1, robot.number_of_nodes*3) + I);

N = null(error);
O = orth(error);

disp(['size of N: ', mat2str( size(N) )])
disp(['size of O: ', mat2str( size(O) )])

%null(error) - space of all accelerations which can be avoided under any
%external forces.

%orth(error) - space of all accelerations which can't be avoided under
%certain external forces