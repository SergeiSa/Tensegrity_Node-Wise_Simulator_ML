function CoM = get_CoM_v2(masses, r)
r = reshape(r, 3, []);
CoM = r * reshape(masses, [], 1) / sum(masses);
end