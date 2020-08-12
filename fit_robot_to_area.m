function [new_nodes_position, shift] = fit_robot_to_area(nodes_position, A, b)

nodes_position = reshape(nodes_position, 3, []);
n = size(nodes_position, 2);

cvx_begin
variables x(3);
minimize( norm(x) );
subject to
    for i = 1:n
        A*(nodes_position(:, i) + x) <= b;
    end
cvx_end

shift = x;
new_nodes_position = nodes_position + x;
end