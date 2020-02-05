function f_rotors = quadrotor_generate_forces_on_active_nodes(rotors_set, nodes_position, norm_f_array, active_nodes_indices)

f_rotors = zeros(3, length(active_nodes_indices));
for i = 1:length(rotors_set)
    [f, application_node_index] = rotors_set{i}(nodes_position, norm_f_array(i));
    f_rotors(:, active_nodes_indices == application_node_index) = f;
end

end