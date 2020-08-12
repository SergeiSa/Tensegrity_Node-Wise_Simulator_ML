function x = solve_FK_fmincon_floatin_base(robot, ro, x0, varargin)

Parser = inputParser;
Parser.FunctionName = 'solve_FK_fmincon_floatin_base';
Parser.addOptional('options', optimoptions('fmincon', 'Display', 'none'));
Parser.parse(varargin{:});

node_position_0 = reshape(x0, 3, []);

%%%Cost design - begin
potential_energy_header = @(x) get_potential_energy(robot.Connectivity, reshape(x, 3, []), robot.stiffness_coef, ro);

%regurilize for rotations
regularization_header = get_floating_base_Z_regularization('indices', robot.z_regularization_indices, 'nodes_position', node_position_0);     
        
cost_weight_potential_enegy = 1; 
cost_weight_regulirization = 0.01;
cost = @(x) cost_weight_potential_enegy * potential_energy_header(x) + ...
            cost_weight_regulirization   * regularization_header(x);
%%%Cost design - end

%%%Constraints design - begin
%constrain center of mass
CoM_0 = get_CoM_v2(robot.nodes_masses, node_position_0);        
NONLCON = @(x) deal([], get_CoM_v2(robot.nodes_masses, x) - CoM_0);    
%%%Constraints design - end                       
                                     
x = fmincon(cost, node_position_0, [],[],[],[],[],[], NONLCON, Parser.Results.options);
end