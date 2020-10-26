function Output = ParameterEstimation_GenerateRegressor_Bodies(robot, Suffix)

ToCheckResiduals = true;

if nargin < 2
    Suffix = 'doe';
end
    
n = size(robot.nodes_position, 2);
m = length(robot.Bodies);

%%%%%%%%%%%%%%%%%%%
% set up known and unknown cable parameters

% the deal here is that some of the cables are known, and some not. The
% known cables are used as a source of the external forces for the
% identification

known_mu = 10;
known_rho = 0.5;

known_cables = zeros(robot.number_of_nodes, robot.number_of_nodes);
known_cables(1, 4) = 1;
known_cables(2, 5) = 1;
known_cables(3, 6) = 1;

known_cables = known_cables + known_cables';
if max(known_cables(:)) > 1
    error('Something went wrong!')
end

known_cables_mu = known_cables * known_mu;
known_cables_rho = known_cables * known_rho;

Cables = robot.Cables;
Cables = Cables - known_cables;

if min(Cables(:)) < 0
    error('Something went wrong!')
end

%%%%%%%%%%%%%%%%%%%
% get parameters

structure_mu = optimization_generate_vector(Cables, 'mu');
structure_rho = optimization_generate_vector(Cables, 'rho');

%%%%%%%%%%%%%%%%%%%
% get variables
r   = sym('r',   [3, n]);     assume(r,   'real');
dr  = sym('dr',  [3, n]);     assume(dr,  'real');
ddr = sym('ddr', [3, n]);     assume(ddr, 'real');

mass = sym('m', [n, 1]);     assume(mass, 'real');


%%%%%%%%%%%%%%%%%%%
% find elastic forces

compaund_mu  = structure_mu.C  + known_cables_mu;
compaund_rho = structure_rho.C + known_cables_rho;

f_array = get_elastic_force_sums_nodes(robot.Connectivity, r, ...
                                         compaund_mu, compaund_rho);
% f_array = f_array(:, robot.active_nodes);
% f_array = f_array(:);
f_array = simplify(f_array); 

% we treat equation m d^2(x)/dt^2 = sum of forces as
% newton_eq_LHS = newton_eq_RHS, same for the euler eq.
newton_eq_LHS = sym(zeros(3, m));
newton_eq_RHS = sym(zeros(3, m));
euler_eq_LHS  = sym(zeros(3, m));
euler_eq_RHS  = sym(zeros(3, m));

for i = 1:m
    indeces = robot.Bodies{i};
    newton_eq_LHS(:, i) = ddr(:, indeces) * mass(indeces);
    newton_eq_RHS(:, i) = sum( f_array(:, indeces), 2 );
    
    for j = 1:length(indeces)
        euler_eq_LHS(:, i) = euler_eq_LHS(:, i) + cross( mass(indeces(j))*r(:, indeces(j)) , dr(:, indeces(j)) );
        euler_eq_RHS(:, i) = euler_eq_RHS(:, i) + cross(r(:, indeces(j)), f_array(:, indeces(j)));
    end
    
end 

newton_eq_LHS = simplify(newton_eq_LHS); 
newton_eq_RHS = simplify(newton_eq_RHS); 
euler_eq_LHS = simplify(euler_eq_LHS); 
euler_eq_RHS = simplify(euler_eq_RHS); 

equations_LHS = [newton_eq_LHS; euler_eq_LHS];
equations_LHS = equations_LHS(:);

equations_RHS = [newton_eq_RHS; euler_eq_RHS];
equations_RHS = equations_RHS(:);


%%%%%%%%%%%%%%%%%
%calculate external forces due to known elastic elements (see description
%above
external_forces = subs(equations_RHS,   structure_mu.var,  zeros(size(structure_mu.var)));
external_forces = subs(external_forces, structure_rho.var, zeros(size(structure_rho.var)));

equations_RHS = equations_RHS - external_forces;
external_forces = simplify(external_forces);
equations_RHS = simplify(equations_RHS);

%%%%%%%%%%%%%%%%%

equations_LHS_regressor = jacobian(equations_LHS, mass);

if ToCheckResiduals
    equations_LHS_regressor_residual = equations_LHS - equations_LHS_regressor*mass;
    disp('Parameter linearization residual (SHOULD BE 0):')
    equations_LHS_regressor_residual = simplify(equations_LHS_regressor_residual)
end
%%%%%%%%%%%%%%%%%

equations_RHS_regressor = [];
for i = 1:length(equations_RHS)
    [coef_bilinear_mu_rho, coef_linear_mu, var_mu_rho, var_mu] = ...
        get_regressor_coefficients(equations_RHS(i), structure_mu.var, structure_rho.var);
    
    equations_RHS_regressor = [equations_RHS_regressor; [coef_bilinear_mu_rho', coef_linear_mu']];
end
%%%%%%%%%%%%%%%%%

regressor = [equations_LHS_regressor, -equations_RHS_regressor];
vars = [mass; var_mu_rho; var_mu];

Output.regressor_size = size(regressor);

%%%%%%%%%%%%%%%%%
Output.symbolic.mass = mass;
Output.symbolic.mu = structure_mu.var;
Output.symbolic.rho = structure_rho.var;



Output.regressor_function_name = ['g_ParameterEstimation_regressor_', Suffix];
Output.regressor_function_handle = matlabFunction(regressor, 'File', Output.regressor_function_name, 'Vars', {r, dr, ddr});
disp(['finished generating', Output.regressor_function_name]);

Output.forces_function_name = ['g_ParameterEstimation_forces_', Suffix];
Output.forces_function_handle = matlabFunction(external_forces, 'File', Output.forces_function_name, 'Vars', {r});
disp(['finished generating', Output.forces_function_name]);

Output.parameters_function_name = ['g_ParameterEstimation_parameters_', Suffix];
Output.parameters_function_handle = matlabFunction(vars, 'File', Output.parameters_function_name, ...
    'Vars', {Output.symbolic.mass, Output.symbolic.mu, Output.symbolic.rho});
disp(['finished generating', Output.parameters_function_name]);

rehash; 


Output.robot = robot;
Output.Suffix = Suffix;

Output.symbolic.vars = vars;
Output.symbolic.regressor = regressor;
Output.symbolic.external_forces = external_forces;
Output.symbolic.r = r;
Output.symbolic.dr = dr;
Output.symbolic.ddr = ddr;
Output.structure_mu = structure_mu;
Output.structure_rho = structure_rho;




% Output.LHS_function_name = ['g_regressor__LHS_true_parameters_', Suffix];
% Output.LHS_function_handle = matlabFunction(f_array, 'File', Output.LHS_function_name, 'Vars', {r});
% disp(['finished generating', Output.LHS_function_name]);
%                                      
% f_array = get_elastic_force_sums_nodes(robot.Connectivity, r, ...
%                                          structure_mu.C, structure_rho.C);                                     
%                                      
% f_array = f_array(:, robot.active_nodes);
% f_array = f_array(:);
% f_array = simplify(f_array);                                     
% 
% regressor = [];
% for i = 1:length(f_array)
%     [coef_bilinear_mu_rho, coef_linear_mu] = get_regressor_coefficients(f_array(i), structure_mu.var, structure_rho.var);
%     
%     regressor = [regressor; [coef_bilinear_mu_rho', coef_linear_mu']];
% end
% 
% Output.regressor_function_name = ['g_regressor__regressor_', Suffix];
% Output.regressor_function_handle = matlabFunction(regressor, 'File', Output.regressor_function_name, 'Vars', {r});
% disp(['finished generating', Output.regressor_function_name]);
% 
% p = structure_mu.var .* structure_rho.var;
% p = [p; structure_mu.var];
% 
% Output.true_p_function_name = ['g_regressor__true_p_', Suffix];
% Output.true_p_function_handle = matlabFunction(p, 'File', Output.true_p_function_name, 'Vars', {structure_mu.var, structure_rho.var});
% disp(['finished generating', Output.true_p_function_name]);
% 
% rehash; 
% 
% Output.robot = robot;
% Output.Suffix = Suffix;
% 
% Output.symbolic.p = p;
% Output.symbolic.regressor = regressor;
% Output.symbolic.LHS = f_array;
% Output.symbolic.r = r;
% Output.structure_mu = structure_mu;
% Output.structure_rho = structure_rho;




function [coef_bilinear_mu_rho, coef_linear_mu, variables_mu_rho, variables_mu] = get_regressor_coefficients(f, var_mu, var_rho)
J_mu = jacobian(f, var_mu);
Q = jacobian(J_mu, var_rho);
coef_bilinear_mu_rho = diag(Q);
coef_bilinear_mu_rho = simplify(coef_bilinear_mu_rho);

coef_linear_mu = J_mu' - Q*var_rho;
coef_linear_mu = simplify(coef_linear_mu);

variables_mu_rho = var_mu .* var_rho;
variables_mu = var_mu;

if ToCheckResiduals
    disp('Parameter linearization residual (SHOULD BE 0):')
    residual = f - ( variables_mu_rho' * coef_bilinear_mu_rho + coef_linear_mu' * variables_mu);
    simplify(residual)
end
end

end