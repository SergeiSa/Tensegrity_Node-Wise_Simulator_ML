function f_array = get_elastic_force_sums_nodes(Connectivity, nodes_position, stiffness_coef, rest_lengths)

if strcmp( class(nodes_position), 'double') && strcmp(class(stiffness_coef), 'double') && strcmp(class(rest_lengths), 'double')
    Type = 'double';
end
if strcmp(class(nodes_position), 'sym') || strcmp(class(stiffness_coef), 'sym') || strcmp(class(rest_lengths), 'sym')
    Type = 'sym';
end
if strcmp(class(nodes_position), 'casadi.SX') || strcmp(class(stiffness_coef), 'casadi.SX') || strcmp(class(rest_lengths), 'casadi.SX')
    Type = 'casadi.SX';
end


switch Type
    case 'double'
        f_array = zeros(3, size(Connectivity, 1));
    case 'sym'
        f_array = sym( zeros(3, size(Connectivity, 1)) );
    case 'casadi.SX'
        import casadi.*
        f_array = SX.zeros(3, size(Connectivity, 1));
end
    


for i = 1:size(Connectivity, 1)
    f = zeros(3, 1);
    for j = 1:size(Connectivity, 2)
        if Connectivity(i, j) == 1
            ri_rj = nodes_position(:, j) - nodes_position(:, i);
            
            valid_force = false; %this construct is here to 1) make sure we don't devide by 0, 
                                 %2) to avoid checking condition (1) on sym computations
            if isnumeric(ri_rj)
                if norm(ri_rj) > 0
                    valid_force = true;
                end
            else
                valid_force = true;
            end
            if valid_force
                f = f + stiffness_coef(i, j) * (norm(ri_rj) - rest_lengths(i, j)) * ...
                    ri_rj / norm(ri_rj);
            end
            
        end  
        
        f_array(:, i) = f; 
    end
end

end