function f_rotors = quadrotor_generate_forces_on_active_nodes(rotors_set, nodes_position, norm_f_array, active_nodes_indices)

f_rotors = zeros(3, length(active_nodes_indices));
for i = 1:length(rotors_set)
    f = rotors_set{i}.rotor_handle(nodes_position, norm_f_array(i));
    if any(active_nodes_indices == rotors_set{i}.application_node_index)
        f_rotors(:, active_nodes_indices == rotors_set{i}.application_node_index) = f;
    else
        error('application_node_index is not in the active_nodes_indices') 
    end
end

end