clc; close all; clear;

Cables = zeros(6, 6);
Cables(1, 2) = 1;
Cables(1, 3) = 1;
Cables(1, 4) = 1;
Cables(2, 3) = 1;
Cables(2, 5) = 1;
Cables(3, 6) = 1;
Cables = math_add_symmetric_components(Cables);


Rods = zeros(6, 6);
Rods(1, 6) = 1;
Rods(2, 4) = 1;
Rods(3, 5) = 1;
Rods = math_add_symmetric_components(Rods);

robot.Connectivity = Cables + Rods;
robot.Cables = Cables;
robot.Rods = Rods;

L_cables = Cables * 0.5;
L_rods = Rods * 2;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 100;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_position = [    0.8660   -0.8660    0         0.6411   -0.2979   -0.3139
                            0.5000    0.5000   -1.0000    0.0154    0.5613   -0.5585
                            0         0         0         1.0365    1.0277    1.0347];
robot.nodes_velocity = zeros(3, size(robot.Connectivity, 1));
robot.nodes_masses = ones(size(robot.Connectivity, 1), 1);
robot.nodes_dissipation = ones(size(robot.Connectivity, 1), 1);



% r = [    1         -1        0         0.5   -0.5   -0.5
%          0.5000    0.5000   -1.0000    0     0.5    -0.5
%          0         0         0         1     1       1];

rotor_handle1 = quadrotor_set_rotor(1, get_second_node_connected_to_rod(robot, 1), eye(3));
rotor_handle2 = quadrotor_set_rotor(2, get_second_node_connected_to_rod(robot, 2), eye(3));
rotor_handle3 = quadrotor_set_rotor(3, get_second_node_connected_to_rod(robot, 3), eye(3));
     
rotors_set = {rotor_handle1, rotor_handle2, rotor_handle3};

%%%%%%%%%%%%%%%%%%%%%%

[f1, application_node_index1] = rotor_handle1(robot.nodes_position, 0.5);
[f2, application_node_index2] = rotor_handle2(robot.nodes_position, 0.5);
[f3, application_node_index3] = rotor_handle3(robot.nodes_position, 0.5);   
     
figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position);

node1 = reshape(robot.nodes_position(:, application_node_index1), [1, 3]);
node2 = reshape(robot.nodes_position(:, application_node_index2), [1, 3]);
node3 = reshape(robot.nodes_position(:, application_node_index3), [1, 3]);
mArrow3(node1, node1 + reshape(f1, [1, 3]), 'facealpha', 0.5, 'color', 'red', 'stemWidth', 0.02);
mArrow3(node2, node2 + reshape(f2, [1, 3]), 'facealpha', 0.5, 'color', 'red', 'stemWidth', 0.02);
mArrow3(node3, node3 + reshape(f3, [1, 3]), 'facealpha', 0.5, 'color', 'red', 'stemWidth', 0.02);

axis equal;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
active_nodes_indices = [1, 2, 3];
Res = quadrotor_Simulate(robot, 3, 10^(-3), active_nodes_indices, rotors_set);

figure_handle = figure('Color', 'w');
plot(Res.Time, Res.Position(:, [1, 2, 3]), 'LineWidth', 2, 'LineStyle', '-'); hold on;
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$x_1$$, $$y_1$$, $$z_1$$ (m)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$x_1$$ (m)', '$$y_1$$ (m)', '$$z_1$$ (m)');
legend_handle.Interpreter = 'latex';


figure_handle = figure('Color', 'w');
plot3(Res.Position(:, 1), Res.Position(:, 2), Res.Position(:, 3), 'LineWidth', 2, 'LineStyle', '-'); hold on;
plot3(Res.Position(:, 4), Res.Position(:, 5), Res.Position(:, 6), 'LineWidth', 2, 'LineStyle', '-'); hold on;
plot3(Res.Position(:, 7), Res.Position(:, 8), Res.Position(:, 9), 'LineWidth', 2, 'LineStyle', '-'); hold on;
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$r_i$$ (m)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$r_1$$', '$$r_2$$', '$$r_1$$');
legend_handle.Interpreter = 'latex';

figure_handle = figure('Color', 'w');
h = [];
for i = 1:50:size(Res.Position, 1)
    r = [reshape(Res.Position(i, :), [3, 3]), robot.nodes_position(:, 4:6)];
    thrusts = reshape(Res.Thrusts(i, :), [3, size(Res.Thrusts, 2)/3]);
    hold off;
    vis_Draw(robot, r);
    quadrotor_vis_thrusts(r, thrusts, active_nodes_indices);
    
    ax = gca;
    axis equal;
    ax.XLim = [-1.3 1.3];
    ax.YLim = [-1.3 1.3];
    ax.ZLim = [-2 1.1];
    drawnow;
end

