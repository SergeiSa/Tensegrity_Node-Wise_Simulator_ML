% controller_input.r
% controller_input.robot
% controller_input.rotors_set
% controller_input.desired_normal
% controller_input.desired_position
% controller_input.K
% controller_input.K_position
% controller_input.K_orientation
% controller_input.betta
function u = quadrotor_Controller_try2(controller_input)

m = length(controller_input.rotors_set);
application_node_indeces = zeros(m, 1);
second_node_indeces = zeros(m, 1);

P = zeros(3, m);
second_node_P = zeros(3, m);
D = zeros(3, m);

for i = 1:length(controller_input.rotors_set)
    application_node_indeces(i) = controller_input.rotors_set{i}.application_node_index;
    second_node_indeces(i) = get_second_node_connected_to_rod(controller_input.robot, application_node_indeces(i));
    
    P(:, i) = controller_input.r(:, application_node_indeces(i));
    second_node_P(:, i) = controller_input.r(:, second_node_indeces(i));
    
    D(:, i) = P(:, i) - second_node_P(:, i);
    D(:, i) = D(:, i) / norm(D(:, i));
end

%%%%%%%%%%%%%%%%%%%%%%%%
%%% task
M = [P(1, 1:4)', P(2, 1:4)', ones(4, 1)];
b = P(3, 1:4)';

%k(1)*x + k(2)*y + k(3) = z
k = pinv(M) * b;

%normal
e = [k(1); k(2); 1];
e = e / norm(e);

rC = get_CoM(controller_input.robot, controller_input.r);

error_position = controller_input.desired_position - rC;
err_angle = acos(dot(e, controller_input.desired_normal));
err_axis = cross(e, controller_input.desired_normal);
err_axis = err_axis / norm(err_axis);
error_orientation = err_axis*err_angle;

error_wrench = [error_position; error_orientation];
disp(mat2str(round(error_wrench', 2)));
disp([mat2str(round(controller_input.desired_normal', 2)), ' ---- ', ...
      mat2str(round(e', 2))]);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% robot position

%thrust torque directions
Rd = zeros(size(D));
for i = 1:size(D, 2)
    Rd(:, i) = cross((P(:, i) - rC), D(:, i));
end

% G = [0; 0; sum(controller_input.robot.nodes_masses) * controller_input.robot.g; 0;0;0];
G = [0; 0; sum(controller_input.robot.nodes_masses) * controller_input.robot.g];

gamma_orientation = controller_input.K_orientation * error_orientation;
gamma_position = controller_input.K_position * error_position + G;

% we want to formulate control as follows:
%
% mininmize: ||Rd * u - gamma_orientation|| + betta*||u||
% subject to: D*u = gamma_position;
%
% this is equivalent to:
%
% [(2* Rd'*Rd + betta*I),   -D] * [x      ]  =  [2*gamma_orientation'*Rd        ]
% [ D,                       0]   [lambda']     [gamma_position]

w1 = 1;
w2 = 1;
w3 = 1;

ControlMatrix = w1*D'*D + w2*Rd'*Rd + w3*eye(m);

solution = pinv(ControlMatrix) * (gamma_position'*D + gamma_orientation'*Rd)';

u = solution(1:m);

% ControlMatrix = [D; Rd];
% u = pinv(ControlMatrix)* (controller_input.K * error_wrench + G);
1;
% u = ones(length(rotors_set), 1);

end