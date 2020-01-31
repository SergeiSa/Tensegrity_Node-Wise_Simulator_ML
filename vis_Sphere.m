function h = vis_Sphere(position, radius, varargin)
Parser = inputParser;
Parser.FunctionName = 'MyFnc';
Parser.addOptional('EdgeAlpha', 0);
Parser.addOptional('FaceColor', [1 0.2 1]);
Parser.addOptional('SpecularStrength', 0.2);
Parser.parse(varargin{:});

[sphere_x, sphere_y, sphere_z] = sphere;

sphere_xi = sphere_x*radius + position(1);
sphere_yi = sphere_y*radius + position(2);
sphere_zi = sphere_z*radius + position(3);

h = surf(sphere_xi,sphere_yi,sphere_zi, ...
    'EdgeAlpha', Parser.Results.EdgeAlpha, ...
    'FaceColor', Parser.Results.FaceColor, ...
    'SpecularStrength', Parser.Results.SpecularStrength);

end


