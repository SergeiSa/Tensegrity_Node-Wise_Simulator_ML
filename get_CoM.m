function CoM = get_CoM(robot, r)
r = reshape(r, 3, []);
CoM = r * reshape(robot.nodes_masses, [], 1) / sum(robot.nodes_masses);
end