function h = vis_Draw(robot, r)

node_radius = 0.1;
cables_radius = 0.01;
rods_radius = 0.05;

for i = 1:size(r, 2)
    h.nodes(i) = vis_Sphere(r(:, i), node_radius); hold on;
end

index = 0;
for i = 1:size(robot.Cables, 1)
for j = 1:size(robot.Cables, 2)
if (i < j) && (robot.Cables(i, j) == 1)
    index = index + 1;
    h.cables(index) = vis_Cylinder(r(:, i), r(:, j), cables_radius, 'FaceColor', [0 0.2 0]);
end
end
end

index = 0;
for i = 1:size(robot.Rods, 1)
for j = 1:size(robot.Rods, 2)
if (i < j) && (robot.Rods(i, j) == 1)
    index = index + 1;
    h.rods(index) = vis_Cylinder(r(:, i), r(:, j), rods_radius);
end
end
end


end