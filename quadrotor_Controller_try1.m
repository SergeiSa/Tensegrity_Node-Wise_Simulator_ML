function u = quadrotor_Controller_try1(rotors_set)

for i = 1:length(rotors_set)
    rotors_set{i}.application_node_index;
end



u = ones(length(rotors_set), 1);

end