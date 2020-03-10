clc; close all; clear;

sim_time = 5;
Animate = true;
frames_skipped = 400;

k = 16;
active_nodes_indices = 1:k;

Cables = zeros(k, k);

%upper circle
Cables(1, 2) = 1;
Cables(2, 3) = 1;
Cables(3, 4) = 1;
Cables(4, 5) = 1;
Cables(5, 6) = 1;
Cables(6, 1) = 1;

%lower circle
Cables(11, 12) = 1;
Cables(12, 13) = 1;
Cables(13, 14) = 1;
Cables(14, 15) = 1;
Cables(15, 16) = 1;
Cables(16, 11) = 1;

%upper circle-to-down cables
Cables(1, 7) = 1;
Cables(2, 8) = 1;
Cables(3, 8) = 1;
Cables(3, 9) = 1;
Cables(4, 9) = 1;
Cables(5, 10) = 1;
Cables(6, 10) = 1;
Cables(6, 7) = 1;


%lower circle-to-up cables
Cables(11, 7) = 1;
Cables(12, 8) = 1;
Cables(13, 8) = 1;
Cables(13, 9) = 1;
Cables(14, 9) = 1;
Cables(15, 10) = 1;
Cables(16, 10) = 1;
Cables(16, 7) = 1;

Cables = Cables + Cables';
if max(Cables(:)) > 1
    error('Something went wrong!')
end


Rods = zeros(k, k);
%z direction
Rods(1, 11) = 1;
Rods(2, 12) = 1;
Rods(4, 14) = 1;
Rods(5, 15) = 1;

%y direction
Rods(3, 6) = 1;
Rods(13, 16) = 1;

%x direction
Rods(7, 10) = 1;
Rods(8, 9) = 1;

Rods = Rods + Rods';
if max(Rods(:)) > 1
    error('Something went wrong!')
end



robot.Connectivity = Cables + Rods;
robot.Cables = Cables;
robot.Rods = Rods;

L_cables = Cables * 0.5;
L_rods = Rods * 2;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 500;
mu_rods = Rods * 2000;
robot.stiffness_coef = mu_cables + mu_rods;

nodes_position1 = [-1  1 1.5   1 -1  -1.5;
                   -1 -1 0     1  1   0;
                    1  1 1     1  1   1];
nodes_position2 = [-1.2  1.2  1.2  -1.2;
                   -1.5 -1.5  1.5   1.5;
                    0    0    0     0];
nodes_position3 = [-1  1  1.5   1  -1  -1.5;
                   -1 -1  0     1   1   0;
                   -1 -1 -1    -1  -1  -1];
robot.nodes_position  = [nodes_position1, nodes_position2, nodes_position3];               

% initial_position0 = reshape(Res.Position(end, :), [3, 16]);
% initial_position = reshape(Res.Position(end, :), [3, 16]) - mean(initial_position0')'
% save('quadrotor_data_turtle_initial_position', 'initial_position')

temp = load('quadrotor_data_turtle_initial_position');
robot.nodes_position = temp.initial_position;

robot.nodes_velocity    = zeros(3, size(robot.Connectivity, 1));
robot.nodes_masses      = ones (size(robot.Connectivity, 1), 1)*0.01;
robot.nodes_dissipation = ones (size(robot.Connectivity, 1), 1)*10;
robot.g = 9.81;


rotor_handle1 = quadrotor_set_rotor(1, get_second_node_connected_to_rod(robot, 1), eye(3));
rotor_handle2 = quadrotor_set_rotor(2, get_second_node_connected_to_rod(robot, 2), eye(3));
rotor_handle3 = quadrotor_set_rotor(4, get_second_node_connected_to_rod(robot, 4), eye(3));
rotor_handle4 = quadrotor_set_rotor(5, get_second_node_connected_to_rod(robot, 5), eye(3));
     
rotors_set = {rotor_handle1, rotor_handle2, rotor_handle3, rotor_handle4};

%%%%%%%%%%%%%%%%%%%%%%
%%%%% drawing 

p = robot.nodes_position;

figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position, 'FaceAlpha', 0.10);

text_delta_x = 0.1;
text_delta_z = 0.1;

for i = 1:size(robot.nodes_position, 2)
    text(p(1, i) + text_delta_x, p(2, i), p(3, i) + text_delta_z, ...
        num2str(i), ...
        'FontName', 'Times New Roman', 'FontSize', 18, 'Color', 'k', 'FontWeight', 'bold');
end

indeces = [1, 11; 2, 12; 3, 13; 4, 14; 5, 15; 6, 16];
indeces2 = [7, 10; 8, 9];
indeces3 = [3, 6; 13, 16];

for i = 1:size(indeces, 1)
    plot3([p(1, indeces(i, 1)); p(1, indeces(i, 2))], ...
          [p(2, indeces(i, 1)); p(2, indeces(i, 2))], ...
          [p(3, indeces(i, 1)); p(3, indeces(i, 2))], ...
          'LineWidth', 2, 'Color', 'r', 'LineStyle', ':');
end

% P1 = zeros(3, 1);
% P2 = zeros(3, 1);
% for i = 1:size(indeces, 1)
%     P1 = P1 + p(:, indeces(i, 1));
%     P2 = P2 + p(:, indeces(i, 2));
% end
% P1 = P1 / size(indeces, 1);
% P2 = P2 / size(indeces, 1);
% P_1 = (P1 + P2) / 2;
% P_2 = P_1 + (P1 - P2);

plot_P1P2(indeces,  p, [1 0 0]);
plot_P1P2(indeces2, p, [0 0.5 0]);
plot_P1P2(indeces3, p, [0 0 1]);


axis equal;


ax = gca;
ax.Visible = 'off';

print(figure_handle, '-dpng', 'Orientation', '-r600');



function plot_P1P2(indeces, p, Color)
P1 = zeros(3, 1);
P2 = zeros(3, 1);
for i = 1:size(indeces, 1)
    P1 = P1 + p(:, indeces(i, 1));
    P2 = P2 + p(:, indeces(i, 2));
end
P1 = P1 / size(indeces, 1);
P2 = P2 / size(indeces, 1);
P_1 = (P1 + P2) / 2;
P_2 = P_1 + (P1 - P2);

mArrow3(P_1,P_2,'color',Color,'stemWidth',0.02,'facealpha',0.5)

% plot3([P_1(1); P_2(1)], ...
%     [P_1(2); P_2(2)], ...
%     [P_1(3); P_2(3)], ...
%     'LineWidth', 2, 'Color', Color, 'LineStyle', '-');
end

