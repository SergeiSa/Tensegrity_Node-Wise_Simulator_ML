function rotor_handle = quadrotor_set_rotor(main_node_index, second_node_index, RelativeOrientation)

    function [f, application_node_index] = rotor(nodes_position, norm_f)
        main_node   = nodes_position(:, main_node_index);
        second_node = nodes_position(:, second_node_index);
        n = main_node - second_node;
        
        f = RelativeOrientation*n*norm_f;
        application_node_index = main_node_index;
    end

    rotor_handle = @rotor;
end
