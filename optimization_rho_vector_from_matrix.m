function vector_rho = optimization_rho_vector_from_matrix(matrix_rho, map)

vector_rho = zeros(size(map, 1), 1);

for i = 1:size(map, 1)

    index1 = map(i, 1);
    index2 = map(i, 2);
    
    vector_rho(i) = matrix_rho(index1, index2);    
end

end