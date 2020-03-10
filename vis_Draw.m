function h = vis_Draw(robot, r, varargin)
Parser = inputParser;
Parser.FunctionName = 'vis_Draw';
Parser.addOptional('NodeRadius', 0.1);
Parser.addOptional('CablesRadius', 0.01);
Parser.addOptional('RodsRadius', 0.05);
Parser.addOptional('FaceAlpha', 1);
Parser.parse(varargin{:});

node_radius = Parser.Results.NodeRadius;
cables_radius = Parser.Results.CablesRadius;
rods_radius = Parser.Results.RodsRadius;

for i = 1:size(r, 2)
    h.nodes(i) = vis_Sphere(r(:, i), node_radius, ...
        'FaceAlpha', Parser.Results.FaceAlpha); 
    hold on;
end

index = 0;
for i = 1:size(robot.Cables, 1)
for j = 1:size(robot.Cables, 2)
if (i < j) && (robot.Cables(i, j) == 1)
    index = index + 1;
    h.cables(index) = vis_Cylinder(r(:, i), r(:, j), cables_radius, ...
        'FaceColor', [0 0.2 0], 'FaceAlpha', Parser.Results.FaceAlpha);
end
end
end

index = 0;
for i = 1:size(robot.Rods, 1)
for j = 1:size(robot.Rods, 2)
if (i < j) && (robot.Rods(i, j) == 1)
    index = index + 1;
    h.rods(index) = vis_Cylinder(r(:, i), r(:, j), rods_radius, ...
        'FaceAlpha', Parser.Results.FaceAlpha);
end
end
end


end