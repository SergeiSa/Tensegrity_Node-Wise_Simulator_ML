function ParameterEstimation_EvaluateRegressor_bodies(RegressorStructure)

robot = RegressorStructure.robot;

%%%%%%%%%%%%%%%%%%%
% get parameters
structure_mu = RegressorStructure.structure_mu;
structure_rho = RegressorStructure.structure_rho;



ind = sub2ind(size(robot.stiffness_coef),structure_mu.map(:, 1),structure_mu.map(:, 2));
value_mu = robot.stiffness_coef(ind);

ind = sub2ind(size(robot.rest_lengths),structure_rho.map(:, 1),structure_rho.map(:, 2));
value_rho = robot.rest_lengths(ind);

value_mass = robot.nodes_masses;

true_value_p = RegressorStructure.parameters_function_handle(value_mass, value_mu, value_rho);
% true_value_p = g_regressor__true_p(value_mu, value_rho);

Count = 100; 

compound_regressor = zeros(Output.regressor_size(1)*Count, Output.regressor_size(2)); 
compound_forces = zeros(Output.regressor_size(1)*Count, 1); 

variation = 0.1;

for i = 1:Count
r = robot.nodes_position;
dr = zeros(size(r));
ddr = zeros(size(r));

r(:, robot.active_nodes) = r(:, robot.active_nodes) + randn( size(r(:, robot.active_nodes)) ) * variation;
dr(:, robot.active_nodes) = r(:, robot.active_nodes) + randn( size(r(:, robot.active_nodes)) ) * variation;

LHS = RegressorStructure.LHS_function_handle(r);
% LHS = g_regressor__f_array_true_parameters(r);

M = RegressorStructure.regressor_function_handle(r);
% M = g_regressor__regressor(r);

index = (i-1)*mm + 1;
compundM(index:(index+mm-1), :) = M;
compundLHS(index:(index+mm-1), :) = LHS;
end


estimated_p = pinv(compundM) * compundLHS;

disp(' ');
disp('norm(compundM*estimated_p - compundLHS)')
norm(compundM*estimated_p - compundLHS)

disp('size(compundM)')
size(compundM)

disp('rank(compundM)')
rank(compundM)

disp('norm(estimated_p - true_value_p)')
norm(estimated_p - true_value_p)

end