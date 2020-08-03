function CoM = get_CoM(robot, r)
if size(r, 1) ~= 3
    nodes = reshape(r, 3, []);
else
    nodes = r;
end
CoM = nodes * robot.nodes_masses / sum(robot.nodes_masses);
end