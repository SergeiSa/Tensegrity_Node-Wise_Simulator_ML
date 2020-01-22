function result = optimization_generate_rho_vector_and_function(C)

result = optimization_generate_rho_vector(C);

result.function_header = matlabFunction(result.C, 'File', 'g_rest_lengths', 'Vars', {result.rho});

end
