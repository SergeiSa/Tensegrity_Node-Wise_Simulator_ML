function CoM = get_CoM(robot, r)
CoM = r * robot.nodes_masses / sum(robot.nodes_masses);
end