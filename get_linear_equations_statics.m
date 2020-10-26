function result = get_linear_equations_statics(varargin)
Parser = inputParser;
Parser.FunctionName = 'get_linear_equations_statics';
Parser.addOptional('RobotPath', []);
Parser.addOptional('robot', []);
Parser.addOptional('nodes_position', []);
Parser.addOptional('external_forces', []);
Parser.parse(varargin{:});

if isempty(Parser.Results.robot)
    robot_data = load(Parser.Results.RobotPath);
    robot = robot_data.robot;
else
    robot = Parser.Results.robot;
end

if isempty(Parser.Results.nodes_position)
    r = robot.nodes_position;
else
    r = Parser.Results.nodes_position;
end

if isempty(Parser.Results.external_forces)
    external_forces = zeros(size(r));
else
    external_forces = Parser.Results.external_forces; %needs to be balanced
end

cable_data = localhelper_get_connection_data(robot.Cables);
rod_data = localhelper_get_connection_data(robot.Rods);


result.A = zeros(robot.number_of_nodes*3, cable_data.number_of_connections);
result.b = zeros(robot.number_of_nodes*3, 1);

for i = 1:cable_data.number_of_connections
    node_index1 = cable_data.map(i, 1);
    node_index2 = cable_data.map(i, 2);
    
    node_position1 = r(:, node_index1);
    node_position2 = r(:, node_index2);
    
    force_direction1 = node_position2 - node_position1; force_direction1 = force_direction1 / norm(force_direction1);
    force_direction2 = node_position1 - node_position2; force_direction2 = force_direction2 / norm(force_direction2);
    
    A_indeces_1 = ((node_index1-1)*3+1):((node_index1-1)*3+3);
    A_indeces_2 = ((node_index2-1)*3+1):((node_index2-1)*3+3);
    
    result.A( A_indeces_1, i ) = force_direction1;
    result.A( A_indeces_2, i ) = force_direction2;
end


for i = 1:rod_data.number_of_connections
    node_index1 = rod_data.map(i, 1);
    node_index2 = rod_data.map(i, 2);
    
    rest_length = robot.rest_lengths(node_index1, node_index2);
    stiffness_coef = robot.stiffness_coef(node_index1, node_index2);
    
    node_position1 = r(:, node_index1);
    node_position2 = r(:, node_index2);
    
    distance = norm(node_position1 - node_position2);
    
    force_direction1 = node_position2 - node_position1; force_direction1 = force_direction1 / norm(force_direction1);
    force_direction2 = node_position1 - node_position2; force_direction2 = force_direction2 / norm(force_direction2);
    
    force1 = force_direction1*(distance - rest_length)*stiffness_coef;
    force2 = force_direction2*(distance - rest_length)*stiffness_coef;
    
    b_indeces_1 = ((node_index1-1)*3+1):((node_index1-1)*3+3);
    b_indeces_2 = ((node_index2-1)*3+1):((node_index2-1)*3+3);
    
    result.b( b_indeces_1 ) = result.b( b_indeces_1 ) + force1;
    result.b( b_indeces_2 ) = result.b( b_indeces_2 ) + force2;
end

result.b = result.b + external_forces(:);
    


function data = localhelper_get_connection_data(Connection)
n = size(Connection, 1);

ind = helper_get_upper_right_courner_incides(n);
data.Connection = Connection;
data.Connection(~ind) = 0;
data.c = data.Connection(:);
data.number_of_connections = sum(data.c > 0);

data.map = zeros(data.number_of_connections, 2);

index = 0;
for local_i = 1:n
    for local_j = 1:n
        if data.Connection(local_i, local_j) > 0
            index = index + 1;
            data.map(index, :) = [local_i, local_j];
        end
    end
end

end

end