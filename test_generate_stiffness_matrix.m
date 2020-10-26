clear; %clear classes;

% FileName_robot = 'data_robot_SixBar_floating_all_cables';
FileName_robot = 'data_robot_SixBar_floating';
robot_data = load(FileName_robot);
robot = robot_data.robot;


result = generate_stiffness_matrix_casadi(robot);


rho_handler = optimization_generate_rho_vector_and_function(robot.Cables);

rho = rho_handler.rho_vector_from_matrix(robot.rest_lengths);
r = robot.nodes_position;

S0 = result.stiffness_func(r, rho);
N0 = null(S0);
O0 = orth(S0);
disp(['size of S0: ', mat2str( size(S0) )])
disp(['size of N0: ', mat2str( size(N0) )])
disp(['size of O0: ', mat2str( size(O0) )])
disp(' ')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = robot.number_of_nodes*3;
m = (robot.number_of_nodes*3)^2;

delta = 0.001;

S_flat_tensor = zeros( m, n );

for i = 1:n
    r = robot.nodes_position;
    r = r(:);
    r(i) = r(i) + delta;
    r = reshape(r, 3, []);
    
    S = result.stiffness_func(r, rho);
    S_flat_tensor(:, i) = S(:);
end

N = null(S_flat_tensor);
O = orth(S_flat_tensor);
disp(['size of S_flat_tensor: ', mat2str( size(S_flat_tensor) )])
disp(['size of N: ', mat2str( size(N) )])
disp(['size of O: ', mat2str( size(O) )])
disp(' ')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% error when getting arbitrary stiffness

s_desired = randn(m, 1);

error = s_desired - S_flat_tensor*(pinv(S_flat_tensor)*s_desired);
disp(['error norm when getting arbitrary stiffness: ', num2str( norm(error) )])
disp(['s_desired norm when getting arbitrary stiffness: ', num2str( norm(s_desired) )])
disp(' ')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% getting one element of stiffness matrix exactly

error = zeros(m, 1);

for index = 1:m
C = zeros(1, m);
C(index) = 1;

s_desired_delta = 0.1;
S0 = result.stiffness_func(robot.nodes_position, rho);
S0 = S0(:);
s_desired = S0(index) + s_desired_delta;

r_desired_delta = pinv(C*S_flat_tensor)*s_desired_delta;
r_desired = robot.nodes_position(:) + r_desired_delta;
r_desired = reshape(r_desired, 3, []);

S1 = result.stiffness_func(r_desired, rho);

error(index) = C*S1(:) - s_desired;

% C*S_flat_tensor*r_desired_delta - s_desired_delta

end

min(abs(error))
% error














