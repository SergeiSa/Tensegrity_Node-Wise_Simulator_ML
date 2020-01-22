function P = get_potential_energy(Connectivity, nodes_position, stiffness_coef, rest_lengths)

P = 0;
for i = 1:size(Connectivity, 1)
    for j = 1:size(Connectivity, 2)
        if Connectivity(i, j) == 1
            ri_rj = nodes_position(:, i) - nodes_position(:, j);
            
            P = P + stiffness_coef(i, j) * (norm(ri_rj) - rest_lengths(i, j))^2;
        end        
    end
end
end