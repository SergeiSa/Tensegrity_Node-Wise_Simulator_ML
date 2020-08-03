function CoM = get_CoM_v2(masses, r)
if size(r, 1) ~= 3
    nodes = reshape(r, 3, []);
else
    nodes = r;
end
CoM = nodes * masses / sum(masses);
end