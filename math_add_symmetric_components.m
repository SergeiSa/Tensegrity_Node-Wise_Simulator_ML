function B = math_add_symmetric_components(A)

B = A;

for i = 1:size(A, 1)
    for j = 1:size(A, 2)
        if i < j
            B(j, i) = A(i, j);
        end
    end
end


end