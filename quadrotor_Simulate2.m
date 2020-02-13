function Res = quadrotor_Simulate2(robot, Time, dt, active_nodes_indices, rotors_set)

number_of_nodes = length(active_nodes_indices);
number_of_rotors = length(rotors_set);

n = number_of_nodes * 3;
m = number_of_rotors * 3;

Count = floor(Time / dt);

Res.Position = zeros(Count, n);
Res.Velocity = zeros(Count, n);
Res.Time = zeros(Count, 1);
Res.Thrusts = zeros(Count, n);
Res.CoM = zeros(Count, 3);
Res.CoM_desired = zeros(Count, 3);
Res.axis = zeros(Count, 3);
Res.axis_desired = zeros(Count, 3);

nodes_position = robot.nodes_position;
r = robot.nodes_position(:, active_nodes_indices);
v = robot.nodes_velocity(:, active_nodes_indices);
masses = robot.nodes_masses(active_nodes_indices);
dissipation = robot.nodes_dissipation(active_nodes_indices);


K_position = 15;
K_orientation = 100;

% rC_0 = get_CoM(robot, r);
% rC_f = rC_0+[0.4; 0; 1];
% control_input = quadrotor_get_control_input_fnc(rC_0, rC_f, Time);

n_0 = [0; 0; 1];
n_f = roty(20)*n_0;
control_input = quadrotor_get_control_input_direction_fnc(n_0, n_f, Time);


for i = 1:Count
    
    t = i*dt;
    
    if rem((t / Time), 0.01) == 0
        disp(['done ', num2str(100* t / Time), '%']);
    end
    
    %elastic forces
    nodes_position(:, active_nodes_indices) = r;
    f_array_elastic = get_elastic_force_sums_nodes(robot.Connectivity, nodes_position, robot.stiffness_coef, robot.rest_lengths);
    f_array_elastic = f_array_elastic(:, active_nodes_indices);
    
    %dissipative forces
    f_array_dissipation = -v;
    for j = 1:length(active_nodes_indices)
        f_array_dissipation(:, j) = f_array_dissipation(:, j) * dissipation(j);
    end
    
%     [desired_position, desired_normal] = control_input(t);
    desired_normal = control_input(t);
    
    controller_input.r = nodes_position;
    controller_input.robot = robot;
    controller_input.rotors_set = rotors_set;
%     controller_input.desired_position = desired_position;
    controller_input.desired_normal = desired_normal;
%     controller_input.K = eye(6) * p_coef;
    controller_input.acc = 1;
    
    
    controller_input.K_position = eye(3) * K_position;
    controller_input.K_orientation = eye(3) * K_orientation;
    controller_input.betta = 0.1;
    
%     u = quadrotor_Controller_try1(controller_input);
%     u = quadrotor_Controller_try2(controller_input);
    u = quadrotor_Controller_try3(controller_input);
%     [u, current_axis] = quadrotor_Controller_try4(controller_input);
    
    f_rotors = quadrotor_generate_forces_on_active_nodes(rotors_set, nodes_position, u, active_nodes_indices);
    
    f_gravity = [zeros(2, number_of_nodes);
                 -robot.g * reshape(robot.nodes_masses, [1, number_of_nodes])];
    
    a = f_array_elastic + f_array_dissipation + f_rotors + f_gravity;
    for j = 1:length(active_nodes_indices)
        a(:, j) = a(:, j) / masses(j);
    end
    
    v = v + a*dt;
    r = r + v*dt + 0.5*a*dt^2;
    
    Res.Position(i, :) = r(:);
    Res.Velocity(i, :) = v(:);
    Res.Time(i) = t;
    Res.Thrusts(i, :) = f_rotors(:);
    
    Res.CoM(i, :) = get_CoM(robot, r);
%     Res.CoM_desired(i, :) = desired_position;
    Res.axis(i, :) = current_axis;
    Res.axis_desired(i, :) = desired_normal;
end

end