function result = optimization_generate_vector(C, varname)

n = (size(C, 1)^2 - size(C, 1)) / 2;

result.map = zeros(n, 2);
result.var = sym(varname, [n, 1]);
assume(result.var, 'real');
result.C = sym(zeros(size(C)));

index = 0;
for i = 1:size(C, 1)
    for j = 1:size(C, 2)
        if (i < j) && (C(i, j) == 1)
            
            index = index + 1;
            result.map(index, :) = [i, j];
            
            result.C(i, j) = result.var(index);
            
        end
    end
end

result.C = result.C + result.C';

result.map = result.map(1:index, :);
result.var = result.var(1:index);
end