clc; close all; clear;


robot.RobotName = 'SixBar';

k = 12;

Cables = zeros(k, k);


Cables(1, 5) = 1;
Cables(1, 6) = 1;
Cables(1, 9) = 1;
Cables(1, 11) = 1;

Cables(3, 5) = 1;
Cables(3, 6) = 1;
Cables(3, 10) = 1;
Cables(3, 12) = 1;


Cables(2, 7) = 1;
Cables(2, 8) = 1;
Cables(2, 9) = 1;
Cables(2, 11) = 1;

Cables(4, 7) = 1;
Cables(4, 8) = 1;
Cables(4, 10) = 1;
Cables(4, 12) = 1;


Cables(7, 9) = 1;
Cables(7, 10) = 1;

Cables(5, 9) = 1;
Cables(5, 10) = 1;

Cables(8, 11) = 1;
Cables(8, 12) = 1;

Cables(6, 11) = 1;
Cables(6, 12) = 1;




Cables = Cables + Cables';
if max(Cables(:)) > 1
    error('Something went wrong!')
end


Rods = zeros(k, k);
Rods(1, 2) = 1;
Rods(3, 4) = 1;
Rods(5, 6) = 1;
Rods(7, 8) = 1;
Rods(9, 10) = 1;
Rods(11, 12) = 1;


Rods = Rods + Rods';
if max(Rods(:)) > 1
    error('Something went wrong!')
end


robot.Connectivity = Cables + Rods;
robot.Cables = Cables;
robot.Rods = Rods;

robot.active_nodes = [1:k]';


L_cables = Cables * 0.25;
L_rods = Rods * 0.5;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 100;
% mu_cables = Cables * 500;
% mu_rods = Rods * 2000;
robot.stiffness_coef = mu_cables + mu_rods;


nodes_position1 = [-0.5 -0.5  0.5 0.5;
                    0  0  0  0;
                    -1 1 -1  1];
                
nodes_position2 = [ 0  0  0  0;
                   -1 1 -1  1;
                   -0.5 -0.5  0.5 0.5];
                
nodes_position3 = [-1 1 -1  1;
                   -0.5 -0.5  0.5 0.5;
                    0  0  0  0];                
                
robot.nodes_position  = [nodes_position1, nodes_position2, nodes_position3]*0.5;               

get_potential_energy_fnc_header = ...
    get_potential_energy_fmincon_wrapper(robot.Connectivity, robot.nodes_position, ...
                                         robot.stiffness_coef, robot.rest_lengths, robot.active_nodes);
x = fminunc(get_potential_energy_fnc_header, robot.nodes_position(:, robot.active_nodes));

robot.nodes_position(:, robot.active_nodes) = x;

f_array = get_elastic_force_sums_nodes(robot.Connectivity, robot.nodes_position, ...
                                         robot.stiffness_coef, robot.rest_lengths)


robot.active_nodes = [2, 4, 7:8, 9:12]';


% initial_position0 = reshape(Res.Position(end, :), [3, 16]);
% initial_position = reshape(Res.Position(end, :), [3, 16]) - mean(initial_position0')'
% save('data_turtle_initial_position', 'initial_position')

%temp = load('data_turtle_initial_position');
%robot.nodes_position = temp.initial_position;

% robot.nodes_velocity    = zeros(3, size(robot.Connectivity, 1));
% robot.nodes_masses      = ones (size(robot.Connectivity, 1), 1)*0.01;
% robot.nodes_dissipation = ones (size(robot.Connectivity, 1), 1)*10;
% robot.g = 9.81;

save(['data_robot_', robot.RobotName, '.mat'], 'robot')

%%%%%%%%%%%%%%%%%%%%%%
%%%%% drawing 

p = robot.nodes_position;

figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position, 'FaceAlpha', 0.30, ...
    'NodeRadius', 0.05, 'CablesRadius', 0.003, 'RodsRadius', 0.02);

text_delta_x = 0.01;
text_delta_z = 0.01;

for i = 1:size(robot.nodes_position, 2)
    text(p(1, i) + text_delta_x, p(2, i), p(3, i) + text_delta_z, ...
        num2str(i), ...
        'FontName', 'Times New Roman', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold');
end

axis equal;

xlabel_handle = xlabel('$$x$$, m', 'Interpreter', 'latex');
ylabel_handle = ylabel('$$y$$, m', 'Interpreter', 'latex');
zlabel_handle = zlabel('$$z$$, m', 'Interpreter', 'latex');

% ax = gca;
% ax.Visible = 'off';
% 
% print(figure_handle, '-dpng', 'Orientation', '-r600');

    