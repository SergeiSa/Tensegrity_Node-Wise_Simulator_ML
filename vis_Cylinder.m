function h = vis_Cylinder(A, B, radius, varargin)
Parser = inputParser;
Parser.FunctionName = 'MyFnc';
Parser.addOptional('EdgeAlpha', 0);
Parser.addOptional('FaceColor', [0.3 0.2 1]);
Parser.addOptional('SpecularStrength', 0.2);
Parser.parse(varargin{:});

[cylinder_x, cylinder_y, cylinder_z] = cylinder;

L = norm(B - A);

cylinder_x = cylinder_x*radius;
cylinder_y = cylinder_y*radius;
cylinder_z = cylinder_z*L - 0.5*L;

Center = (A + B) / 2;

n = (B - A) / norm(B - A);
e = [0; 0; 1];

alpha = acos(dot(n, e));
a = cross(e, n);

axang = [reshape(-a, [1, 3]), alpha];
T = axang2rotm(axang);

P = [cylinder_x(:), cylinder_y(:), cylinder_z(:)];
P = P*T;
cylinder_x = reshape(P(:, 1), size(cylinder_x));
cylinder_y = reshape(P(:, 2), size(cylinder_y));
cylinder_z = reshape(P(:, 3), size(cylinder_z));

cylinder_x = cylinder_x + Center(1);
cylinder_y = cylinder_y + Center(2);
cylinder_z = cylinder_z + Center(3);

h = surf(cylinder_x, cylinder_y, cylinder_z, ...
    'EdgeAlpha', Parser.Results.EdgeAlpha, ...
    'FaceColor', Parser.Results.FaceColor, ...
    'SpecularStrength', Parser.Results.SpecularStrength);

end


