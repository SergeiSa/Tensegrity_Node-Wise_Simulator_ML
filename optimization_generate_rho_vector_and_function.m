function result = optimization_generate_rho_vector_and_function(C)

result = optimization_generate_rho_vector(C);

%vector to matrix
result.function_header = matlabFunction(result.C, 'File', 'g_rest_lengths', 'Vars', {result.rho});

result.rho_matrix_from_vector = result.function_header;

result.rho_vector_from_matrix = @(matrix_rho) optimization_rho_vector_from_matrix(matrix_rho, result.map);

%     function rho_vector = matrix_to_vector(rho_matrix)
%         rho_vector = zeros(size(result.map, 1), 1);
%         for i = 1:size(result.map, 1)
%             rho_vector(i) = rho_matrix(result.map(i, 1), result.map(i, 2));
%         end
%     end
% 
% result.matrix_to_vector_header = @matrix_to_vector;

end
