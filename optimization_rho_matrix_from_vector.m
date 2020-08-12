function matrix_rho = optimization_rho_matrix_from_vector(vector_rho, map, robot)

matrix_rho = zeros(robot.number_of_nodes, robot.number_of_nodes);

for i = 1:size(map, 1)

    index1 = map(i, 1);
    index2 = map(i, 2);
    
    matrix_rho(index1, index2) = vector_rho(i);    
end

matrix_rho = matrix_rho + matrix_rho';

end