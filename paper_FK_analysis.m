clear; close all;

% FileName_robot = 'data_robot_SixBar_floating';
FileName_robot = 'data_robot_SixBar_floating_all_cables';
robot_data = load(FileName_robot);
robot = robot_data.robot;


r = robot.nodes_position;
result = get_linear_equations_statics('robot', robot, 'nodes_position', r, 'external_forces', zeros(size(r)));

N = null(result.A);

disp(['size of A: ', mat2str( size(result.A) )])
disp(['size of N: ', mat2str( size(N) )])


%Column space of A is the space of 1) all possible accelerations the system
%can produce, 2) space of all possible forces the system can resist to
%remain statically stable.

%null space of A is the space of all possible combinations cable tensions
%that allow the structure to remain stable
