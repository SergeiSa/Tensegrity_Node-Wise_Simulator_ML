function ParameterEstimation_EvaluateRegressor(RegressorStructure)

robot = RegressorStructure.robot;

n = size(robot.nodes_position, 2);
m = length(robot.active_nodes);

%%%%%%%%%%%%%%%%%%%
% get parameters
structure_mu = RegressorStructure.structure_mu;
structure_rho = RegressorStructure.structure_rho;



ind = sub2ind(size(robot.stiffness_coef),structure_mu.map(:, 1),structure_mu.map(:, 2));
value_mu = robot.stiffness_coef(ind);

ind = sub2ind(size(robot.rest_lengths),structure_rho.map(:, 1),structure_rho.map(:, 2));
value_rho = robot.rest_lengths(ind);

true_value_p = RegressorStructure.true_p_function_handle(value_mu, value_rho);
% true_value_p = g_regressor__true_p(value_mu, value_rho);

Count = 100; mm = m*3; 
compundM = zeros(mm*Count, length(value_mu)+length(value_rho)); 
compundLHS = zeros(mm*Count, 1); 

for i = 1:Count
r = robot.nodes_position;
r(:, robot.active_nodes) = r(:, robot.active_nodes) + randn(3, m) * 0.1;

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