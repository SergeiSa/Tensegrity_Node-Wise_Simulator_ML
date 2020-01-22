function get_potential_energy_fnc_header = ...
    get_potential_energy_fmincon_wrapper(Connectivity, nodes_position_0, stiffness_coef, rest_lengths, active_nodes_indeces)

    function P = get_potential_energy_fnc(x)
        nodes_position = nodes_position_0;
        
        for i = 1:length(active_nodes_indeces)
            nodes_position(:, active_nodes_indeces(i)) = x(:, i);
        end
            
        P = get_potential_energy(Connectivity, nodes_position, stiffness_coef, rest_lengths);     
    end

get_potential_energy_fnc_header = @get_potential_energy_fnc;

end