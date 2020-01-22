function Res = Simulate(robot, Time, dt, active_nodes_indices)

number_of_nodes = length(active_nodes_indices);
n = number_of_nodes * 3;

Count = floor(Time / dt);

Res.Position = zeros(Count, n);
Res.Velocity = zeros(Count, n);
Res.Time = zeros(Count, 1);

nodes_position = robot.nodes_position;
r = robot.nodes_position(:, active_nodes_indices);
v = robot.nodes_velocity(:, active_nodes_indices);
masses = robot.nodes_masses(active_nodes_indices);
dissipation = robot.nodes_dissipation(active_nodes_indices);

for i = 1:Count
    
    %elastic forces
    nodes_position(:, active_nodes_indices) = r;
    f_array_elastic = get_elastic_force_sums_nodes(robot.Connectivity, nodes_position, robot.stiffness_coef, robot.rest_lengths);
    f_array_elastic = f_array_elastic(:, active_nodes_indices);
    
    %dissipative forces
    f_array_dissipation = -v;
    for j = 1:length(active_nodes_indices)
        f_array_dissipation(:, j) = f_array_dissipation(:, j) / dissipation(j);
    end
    
    a = f_array_elastic + f_array_dissipation;
    for j = 1:length(active_nodes_indices)
        a(:, j) = a(:, j) / masses(j);
    end
    
    v = v + a*dt;
    r = r + v*dt + 0.5*a*dt^2;
    
    Res.Position(i, :) = r(:);
    Res.Velocity(i, :) = v(:);
    Res.Time(i) = i*dt;
end

end