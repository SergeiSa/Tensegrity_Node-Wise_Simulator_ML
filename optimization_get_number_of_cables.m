function index = optimization_get_number_of_cables(robot)

index = 0;
for i = 1:size(robot.Cables, 1)
    for j = 1:size(robot.Cables, 2)
        if (i < j) && (robot.Cables(i, j) == 1)
            index = index + 1;
        end
    end
end

end