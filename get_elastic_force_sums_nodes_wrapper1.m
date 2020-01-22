function f_array = get_elastic_force_sums_nodes_wrapper1(robot)

f_array = get_elastic_force_sums_nodes(robot.Connectivity, robot.nodes_position, robot.stiffness_coef, robot.rest_lengths);

end