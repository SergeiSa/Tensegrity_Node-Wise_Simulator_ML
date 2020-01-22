function f_array = get_elastic_force_sums_nodes(Connectivity, nodes_position, stiffness_coef, rest_lengths)

f_array = zeros(3, size(Connectivity, 1));

for i = 1:size(Connectivity, 1)
    f = zeros(3, 1);
    for j = 1:size(Connectivity, 2)
        if Connectivity(i, j) == 1
            ri_rj = nodes_position(:, j) - nodes_position(:, i);
            
            f = f + stiffness_coef(i, j) * (norm(ri_rj) - rest_lengths(i, j)) * ...
                ri_rj / norm(ri_rj);
        end        
        f_array(:, i) = f; 
    end
end

end