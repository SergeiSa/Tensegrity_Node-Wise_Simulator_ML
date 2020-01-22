function Res = test_C_code_generation()

Cables = zeros(6, 6);
Cables(1, 2) = 1;
Cables(1, 3) = 1;
Cables(1, 4) = 1;
Cables(2, 3) = 1;
Cables(2, 5) = 1;
Cables(3, 6) = 1;
Cables = math_add_symmetric_components(Cables);


Rods = zeros(6, 6);
Rods(1, 6) = 1;
Rods(2, 4) = 1;
Rods(3, 5) = 1;
Rods = math_add_symmetric_components(Rods);

Connectivity = Cables + Rods;

robot.Connectivity = Connectivity;

L_cables = Cables * 0.5;
L_rods = Rods * 2;
robot.rest_lengths = L_cables + L_rods;

mu_cables = Cables * 10;
mu_rods = Rods * 100;
robot.stiffness_coef = mu_cables + mu_rods;

robot.nodes_position = [    0.8660   -0.8660    0         0.6411   -0.2979   -0.3139
                            0.5000    0.5000   -1.0000    0.0154    0.5613   -0.5585
                            0         0         0         1.0365    1.0277    1.0347];
robot.nodes_velocity = zeros(3, size(Connectivity, 1));
robot.nodes_masses = ones(size(Connectivity, 1), 1);
robot.nodes_dissipation = ones(size(Connectivity, 1), 1);

Res = Simulate(robot, 10, 10^(-3), [1, 2, 3]);
end