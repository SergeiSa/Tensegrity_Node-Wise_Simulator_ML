function second_node_index = get_second_node_connected_to_rod(robot, first_node_index)

for i = 1:size(robot.Rods, 2)
    if robot.Rods(first_node_index, i) == 1
        second_node_index = i;
    end
end

end