function result = generate_stiffness_matrix_casadi(robot)

import casadi.*

rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);


r   = SX.sym('r',   size(robot.nodes_position));
rho = SX.sym('rho', size(rho_handler.rho));


rho_matrix = rho_handler.rho_matrix_from_vector(rho);

% r = sym('r', size(robot.nodes_position)); assume(r, 'real');
% rho = rho_handler.rho;

f_array = get_elastic_force_sums_nodes(robot.Connectivity, r, robot.stiffness_coef, ...
    rho_matrix);
%rho_handler.C - symbolic rho matrix

S = jacobian(f_array(:), r(:));

stiffness_func = Function('stiffness_func', {r, rho}, {S}, {'r', 'rho'}, {'S'});

C = CodeGenerator('gen_stiffness_func.c');
C.add(stiffness_func);
C.generate();

!gcc -fPIC -shared gen_stiffness_func.c -o gen_stiffness_func.so
fe = external('stiffness_func', './gen_stiffness_func.so');

result.stiffness_func = @(r, rho) full(evalf(fe(r, rho)));

end