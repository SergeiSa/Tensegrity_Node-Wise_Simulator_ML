function get_elastic_force_sums_nodes_fnc_header = get_elastic_force_sums_nodes_wrapper2(robot)

Connectivity = robot.Connectivity;
stiffness_coef = robot.stiffness_coef;

    function f_array = get_elastic_force_sums_nodes_fnc(nodes_position, rest_lengths)
        f_array = get_elastic_force_sums_nodes(Connectivity, nodes_position, stiffness_coef, rest_lengths);
    end
        
get_elastic_force_sums_nodes_fnc_header = @get_elastic_force_sums_nodes_fnc;
end