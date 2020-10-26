function result = generate_stiffness_matrix_symbolic(robot)

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);


r = sym('r', size(robot.nodes_position)); assume(r, 'real');
% rho = rho_handler.rho;

f_array = get_elastic_force_sums_nodes(robot.Connectivity, r, robot.stiffness_coef, ...
    rho_handler.C);
%rho_handler.C - symbolic rho matrix

S = jacobian(f_array(:), r(:));

Math = MathClass;

Math.simplify(S, 'S');
result.S_func = matlabFunction(S, 'File', 'g_stiffness', 'Vars', {r, rho_handler.rho});

end