%draws a half space, given in the form a'*x <= b
%
%length, width - the dimentions of cube drawn
%
%Example: draw_half_space([1; 0; 0], 1, 'FaceColor', 'r')
%
function h = vis_half_space(a, b, varargin)
Parser = inputParser;
Parser.FunctionName = 'vis_half_space';
Parser.addOptional('length', 10);
Parser.addOptional('width', 10);
Parser.addOptional('EdgeAlpha', 0);
Parser.addOptional('FaceAlpha', 0.3);
Parser.addOptional('FaceColor', [0.2 0.3 1]);
Parser.addOptional('SpecularStrength', 0.2);
Parser.parse(varargin{:});

a = reshape(a, 3, 1);
h = norm(a);
a = a / h;

l = Parser.Results.length;
w = Parser.Results.length;

N = null(a');

p = a*b/h;

P = zeros(3, 8);

P(:, 1) =  N(:, 1)*w + p;
P(:, 2) =  N(:, 2)*w + p;
P(:, 3) = -N(:, 1)*w + p;
P(:, 4) = -N(:, 2)*w + p;

P(:, 5:8) = P(:, 1:4) + a*l;

P = P';
k = convhull(P);

h = trisurf(k,P(:, 1),P(:, 2),P(:, 3), ...
    'EdgeAlpha', Parser.Results.EdgeAlpha, ...
    'FaceAlpha', Parser.Results.FaceAlpha, ...
    'FaceColor', Parser.Results.FaceColor, ...
    'SpecularStrength', Parser.Results.SpecularStrength);
end