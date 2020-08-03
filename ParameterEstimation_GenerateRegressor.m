function Output = ParameterEstimation_GenerateRegressor(robot, Suffix)

if nargin < 2
    Suffix = 'doe';
end
    
n = size(robot.nodes_position, 2);
% m = length(robot.active_nodes);

%%%%%%%%%%%%%%%%%%%
% get parameters
structure_mu = optimization_generate_vector(robot.Connectivity, 'mu');
structure_rho = optimization_generate_vector(robot.Connectivity, 'rho');

%%%%%%%%%%%%%%%%%%%
% get variables
r = sym('r', [3, n]);
assume(r, 'real');

f_array = get_elastic_force_sums_nodes(robot.Connectivity, r, ...
                                         robot.stiffness_coef, robot.rest_lengths);
f_array = f_array(:, robot.active_nodes);
f_array = f_array(:);
f_array = simplify(f_array);        

Output.LHS_function_name = ['g_regressor__LHS_true_parameters_', Suffix];
Output.LHS_function_handle = matlabFunction(f_array, 'File', Output.LHS_function_name, 'Vars', {r});
disp(['finished generating', Output.LHS_function_name]);
                                     
f_array = get_elastic_force_sums_nodes(robot.Connectivity, r, ...
                                         structure_mu.C, structure_rho.C);                                     
                                     
f_array = f_array(:, robot.active_nodes);
f_array = f_array(:);
f_array = simplify(f_array);                                     

regressor = [];
for i = 1:length(f_array)
    [coef_bilinear_mu_rho, coef_linear_mu] = get_regressor_coefficients(f_array(i), structure_mu.var, structure_rho.var);
    
    regressor = [regressor; [coef_bilinear_mu_rho', coef_linear_mu']];
end

Output.regressor_function_name = ['g_regressor__regressor_', Suffix];
Output.regressor_function_handle = matlabFunction(regressor, 'File', Output.regressor_function_name, 'Vars', {r});
disp(['finished generating', Output.regressor_function_name]);

p = structure_mu.var .* structure_rho.var;
p = [p; structure_mu.var];

Output.true_p_function_name = ['g_regressor__true_p_', Suffix];
Output.true_p_function_handle = matlabFunction(p, 'File', Output.true_p_function_name, 'Vars', {structure_mu.var, structure_rho.var});
disp(['finished generating', Output.true_p_function_name]);

rehash; 

Output.robot = robot;
Output.Suffix = Suffix;

Output.symbolic.p = p;
Output.symbolic.regressor = regressor;
Output.symbolic.LHS = f_array;
Output.symbolic.r = r;
Output.structure_mu = structure_mu;
Output.structure_rho = structure_rho;




function [coef_bilinear_mu_rho, coef_linear_mu] = get_regressor_coefficients(f, var_mu, var_rho)
J_mu = jacobian(f, var_mu);
Q = jacobian(J_mu, var_rho);
coef_bilinear_mu_rho = diag(Q);
coef_bilinear_mu_rho = simplify(coef_bilinear_mu_rho);

coef_linear_mu = J_mu' - Q*var_rho;
coef_linear_mu = simplify(coef_linear_mu);

% residual = f - ( (var_mu .* var_rho)' * coef_bilinear_mu_rho + coef_linear_mu' * var_mu);
% simplify(residual)
end

end