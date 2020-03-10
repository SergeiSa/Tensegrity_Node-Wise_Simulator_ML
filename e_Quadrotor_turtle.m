clc; close all; clear;

sim_time = 15;
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
    
figure_handle = figure('Color', 'w');
vis_Draw(robot, robot.nodes_position);

axis equal;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Res = quadrotor_Simulate(robot, sim_time, 10^(-3), active_nodes_indices, rotors_set);

% figure_handle = figure('Color', 'w');
% plot(Res.Time, Res.CoM, 'LineWidth', 2, 'LineStyle', '-'); hold on;
% plot(Res.Time, Res.CoM_desired, 'LineWidth', 3, 'LineStyle', ':'); hold on;
% 
% grid on; grid minor;
% ax = gca;
% ax.GridAlpha = 0.6;
% ax.LineWidth = 0.5;
% ax.MinorGridLineStyle = '-';
% ax.MinorGridAlpha = 0.2;
% ax.FontName = 'Times New Roman';
% ax.FontSize = 18;
% xlabel_handle = xlabel('$$t$$, s');
% xlabel_handle.Interpreter = 'latex';
% ylabel_handle = ylabel('$$x_1$$, $$y_1$$, $$z_1$$ (m)');
% ylabel_handle.Interpreter = 'latex';
% legend_handle = legend('$$x_1$$ (m)', '$$y_1$$ (m)', '$$z_1$$ (m)');
% legend_handle.Interpreter = 'latex';


% % figure_handle = figure('Color', 'w');
% % plot(Res.Time, Res.axis, 'LineWidth', 2, 'LineStyle', '-'); hold on;
% % plot(Res.Time, Res.axis_desired, 'LineWidth', 3, 'LineStyle', ':'); hold on;
% % 
% % grid on; grid minor;
% % ax = gca;
% % ax.GridAlpha = 0.6;
% % ax.LineWidth = 0.5;
% % ax.MinorGridLineStyle = '-';
% % ax.MinorGridAlpha = 0.2;
% % ax.FontName = 'Times New Roman';
% % ax.FontSize = 18;
% % xlabel_handle = xlabel('$$t$$, s');
% % xlabel_handle.Interpreter = 'latex';
% % ylabel_handle = ylabel('$$x_1$$, $$y_1$$, $$z_1$$ (m)');
% % ylabel_handle.Interpreter = 'latex';
% % legend_handle = legend('$$x_1$$ (m)', '$$y_1$$ (m)', '$$z_1$$ (m)');
% % legend_handle.Interpreter = 'latex';



figure_handle = figure('Color', 'w');
indices = [1, 2, 4, 5];
for i = 1:length(indices)
plot3(Res.Position(:, (indices(i)*3-2)), Res.Position(:, (indices(i)*3-1)), Res.Position(:, (indices(i)*3-0)), ...
    'LineWidth', 2, 'LineStyle', '-'); hold on;
end
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
indices = [1, 2, 4, 5];
for i = 1:length(indices)
plot(Res.Time, Res.Position(:, (indices(i)*3-2)), ...
    'LineWidth', 1, 'LineStyle', '-'); hold on;
plot(Res.Time, Res.Position(:, (indices(i)*3-1)), ...
    'LineWidth', 2, 'LineStyle', '--'); 
plot(Res.Time, Res.Position(:, (indices(i)*3-0)), ...
    'LineWidth', 2.5, 'LineStyle', ':'); 
end
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



if Animate
figure_handle = figure('Color', 'w');
h = [];
ax = gca;

xlim = [min(min(Res.Position(:, (1:k)*3-2 ))) max(max(Res.Position(:,   (1:k)*3-2)))];
ylim = [min(min(Res.Position(:, (1:k)*3-1 ))) max(max(Res.Position(:,   (1:k)*3-1)))];
zlim = [min(min(Res.Position(:, (1:k)*3   ))) max(max(Res.Position(:, (1:k)*3 )))];
for i = 1:frames_skipped:size(Res.Position, 1)
    r = reshape(Res.Position(i, :), [3, size(Res.Position, 2)/3]);
    thrusts = reshape(Res.Thrusts(i, :), [3, size(Res.Thrusts, 2)/3]);
    hold off;
    vis_Draw(robot, r);
    quadrotor_vis_thrusts(r, thrusts, active_nodes_indices);
    
    axis equal;
    ax.XLim = xlim;
    ax.YLim = ylim;
    ax.ZLim = zlim;
    drawnow;
end
end

