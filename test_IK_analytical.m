clear; clc; close all;

FileName_robot = 'data_robot_SixBar_floating';
robot_data = load(FileName_robot);
robot = robot_data.robot;
r = robot.nodes_position;

m1 = 6;
m2 = size(r, 2) - m1;

external_1 = randn(3, m1);

% sum1 = sum(external_1, 2);
torque1 = zeros(3, m1);
for i = 1:m1
    torque1(:, i) = cross(r(:, i), external_1(:, i));
end


%%% get the external_forces to obey statics condition
cvx_begin
variables x(3, m2);
minimize( norm(x) );
subject to

% torque2 = zeros(3, m2);
for i = 1:m2
    torque2(:, i) = cross(r(:, i+m1), x(:, i));
end
sum(external_1, 2) == -sum(x, 2);
sum(torque1, 2) == -sum(torque2, 2);

cvx_end

external_forces = [external_1, x];

%%% test the external_forces for statics condition
sum(external_forces, 2)
torque = zeros(size(r));
for i = 1:size(r, 2)
    torque(:, i) = cross(r(:, i), external_forces(:, i));
end
sum(torque, 2)




result = get_linear_equations_statics('robot', robot, 'external_forces', external_forces*0);

P = eye(robot.number_of_nodes*3) - result.A * pinv(result.A);
P*result.b
