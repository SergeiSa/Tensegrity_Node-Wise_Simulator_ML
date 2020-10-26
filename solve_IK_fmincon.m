function [r, rho] = solve_IK_fmincon(robot, r0, rho0, varargin)

Parser = inputParser;
Parser.FunctionName = 'solve_IK_fmincon_closest_pose_for_given_one';
Parser.addOptional('options', optimoptions('fminunc', 'Display', 'none'));
Parser.addOptional('CoM', []);
Parser.addOptional('active_nodes', []);
Parser.parse(varargin{:});

rho_handler = optimization_generate_rho_vector_and_function(robot.Connectivity);

if size(rho0, 1) == size(rho0, 2)
    rho0 = rho_handler.rho_vector_from_matrix(rho0)';
end

if isempty(Parser.Results.CoM)
    CoM0 = get_CoM(robot, r0);
else
    CoM0 = reshape(Parser.Results.CoM, 3, 1); 
end

number_of_nodes = robot.number_of_nodes;

if isempty(Parser.Results.active_nodes)
    active_nodes = 1:number_of_nodes;
else
    active_nodes = Parser.Results.active_nodes;
end

x0 = [reshape(r0, [], 1); reshape(rho0, [], 1)];

node_indices = 1:(number_of_nodes*3);
cable_indices = (number_of_nodes*3 + 1):(number_of_nodes*3 + length(rho0));

%%%Cost design - begin
% potential_energy_header = @(x) get_potential_energy(robot.Connectivity, ...
%                                                     reshape(x(node_indices), 3, []), ...
%                                                     robot.stiffness_coef, ...
%                                                     reshape(x(cable_indices), [], 1) );
        
weight_nodes_reg = 0.001;
weight_nodes_active = 10;
weight_cables = 0.01;
weight_force_violation = 1000;
weight_CoM = 0.01;

node_pos_0 = reshape(r0, 3, []);

    function C = Cost(x)
        node_pos = reshape(x(node_indices), 3, []);
        cable_pos = reshape(x(cable_indices), [], 1);
        
        active_node_pos = node_pos(:, active_nodes);
        
        current_CoM = get_CoM(robot, node_pos + node_pos_0);
        
        current_statics_violation = get_elastic_force_sums_nodes(robot.Connectivity, ...
            	node_pos + node_pos_0, ...
                robot.stiffness_coef, ...
                rho_handler.rho_matrix_from_vector(reshape(x(cable_indices), [], 1) + reshape(rho0, [], 1)));
        
        C = ...
        weight_nodes_reg       * norm(node_pos(:)) + ...
        weight_nodes_active    * norm(active_node_pos(:)) + ...
        weight_cables          * norm(cable_pos) + ...
        weight_force_violation * norm(reshape(current_statics_violation, [], 1)) + ...
        weight_CoM             * norm( CoM0 - current_CoM );
    end

% cost = @(x) weight_pos    * norm(reshape(x(node_indices), [], 1)  - reshape(r0, [], 1)) + ...
%             weight_cables * norm(reshape(x(cable_indices), [], 1) - reshape(rho0, [], 1));
% cost = @(x) weight_pos    * norm(reshape(x(node_indices), [], 1)) + ...
%             weight_cables * norm(reshape(x(cable_indices), [], 1)) + ...
%             weight_force_violation * norm( reshape( ...
%                 get_elastic_force_sums_nodes(robot.Connectivity, ...
%             	reshape(x(node_indices), 3, []) + reshape(r0, 3, []), ...
%                 robot.stiffness_coef, ...
%                 rho_handler.rho_matrix_from_vector(reshape(x(cable_indices), [], 1) + reshape(rho0, [], 1))), ...
%                                                    [], 1 ) ) + ...
%            weight_CoM * norm( CoM0 - get_CoM(robot, reshape(x(node_indices), 3, []) + reshape(r0, 3, [])) );
        
        
%%%Cost design - end

% x = fmincon(@Cost, zeros(size(x0)), [],[],[],[],[],[], [], Parser.Results.options);
x = fminunc(@Cost, zeros(size(x0)), Parser.Results.options);

r = reshape(x(node_indices), 3, []) + reshape(r0, 3, []);
rho = reshape(x(cable_indices), [], 1) + reshape(rho0, [], 1);
end