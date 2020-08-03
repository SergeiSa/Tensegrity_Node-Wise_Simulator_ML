
temp = load('data_robot_SixBar');
% temp = load('data_robot_ThreePrizm');

RegressorStructure = ParameterEstimation_GenerateRegressor(temp.robot, temp.robot.RobotName);

%%
ParameterEstimation_EvaluateRegressor(RegressorStructure);