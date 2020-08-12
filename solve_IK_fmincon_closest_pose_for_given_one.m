function [r, rho] = solve_IK_fmincon_closest_pose_for_given_one(robot, r0, rho0, varargin)

Parser = inputParser;
Parser.FunctionName = 'solve_IK_fmincon_closest_pose_for_given_one';
Parser.addOptional('options', optimoptions('fmincon', 'Display', 'none'));
Parser.parse(varargin{:});

rho_handler = optimization_generate_rho_vector_and_function(robot.Connectivity);

if size(rho0, 1) == size(rho0, 2)
    rho0 = rho_handler.rho_vector_from_matrix(rho0)';
end

number_of_nodes = robot.number_of_nodes;

x0 = [reshape(r0, [], 1); reshape(rho0, [], 1)];

node_indices = 1:(number_of_nodes*3);
cable_indices = (number_of_nodes*3 + 1):(number_of_nodes*3 + length(rho0));

%%%Cost design - begin
% potential_energy_header = @(x) get_potential_energy(robot.Connectivity, ...
%                                                     reshape(x(node_indices), 3, []), ...
%                                                     robot.stiffness_coef, ...
%                                                     reshape(x(cable_indices), [], 1) );
        
weight_pos = 1;
weight_cables = 0.01;
weight_force_violation = 100;

% cost = @(x) weight_pos    * norm(reshape(x(node_indices), [], 1)  - reshape(r0, [], 1)) + ...
%             weight_cables * norm(reshape(x(cable_indices), [], 1) - reshape(rho0, [], 1));
cost = @(x) weight_pos    * norm(reshape(x(node_indices), [], 1)) + ...
            weight_cables * norm(reshape(x(cable_indices), [], 1)) + ...
            weight_force_violation * norm( reshape( ...
                get_elastic_force_sums_nodes(robot.Connectivity, ...
            	reshape(x(node_indices), 3, []) + reshape(r0, 3, []), ...
                robot.stiffness_coef, ...
                rho_handler.rho_matrix_from_vector(reshape(x(cable_indices), [], 1) + reshape(rho0, [], 1))), ...
                                                   [], 1 ) );
        
        
%%%Cost design - end


%%%Constraints design - begin
% NONLCON = @(x) deal([], reshape( get_elastic_force_sums_nodes(robot.Connectivity, ...
%                                                               reshape(x(node_indices), 3, []) + reshape(r0, 3, []), ...
%                                                               robot.stiffness_coef, ...
%                                                               rho_handler.rho_matrix_from_vector(reshape(x(cable_indices), [], 1) + reshape(rho0, [], 1))), ...
%                                                               [], 1 ));    
%%%Constraints design - end                       
      
% x = fmincon(cost, zeros(size(x0)), [],[],[],[],[],[], NONLCON, Parser.Results.options);

x = fmincon(cost, zeros(size(x0)), [],[],[],[],[],[], [], Parser.Results.options);

r = reshape(x(node_indices), 3, []) + reshape(r0, 3, []);
rho = reshape(x(cable_indices), [], 1) + reshape(rho0, [], 1);
end