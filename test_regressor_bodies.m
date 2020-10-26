clear; %clear classes;

% FileName_robot = 'data_robot_SixBar_floating_all_cables';
FileName_robot = 'data_robot_ThreePrizm_floating_bodies';
robot_data = load(FileName_robot);
robot = robot_data.robot;

RegressorStructure = ParameterEstimation_GenerateRegressor_Bodies(robot)
save('data_ParameterEstimation_test', 'RegressorStructure');

%%
temp = load('data_ParameterEstimation_test');
ParameterEstimation_EvaluateRegressor_bodies(temp.RegressorStructure);

