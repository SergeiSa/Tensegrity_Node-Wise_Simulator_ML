function h = quadrotor_vis_thrusts(nodes_position, thrusts, active_nodes_indices)

for i = 1:length(active_nodes_indices)
    index = active_nodes_indices(i);
    h(i) = mArrow3(nodes_position(:, index)', nodes_position(:, index)' + 0.3*thrusts(:, i)', ...
        'facealpha', 0.5, 'color', 'red', 'stemWidth', 0.02);
end

end