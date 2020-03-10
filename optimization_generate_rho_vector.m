function result = optimization_generate_rho_vector(C)

n = (size(C, 1)^2 - size(C, 1)) / 2;

result.map = zeros(n, 2);
result.rho = sym('rho', [n, 1]);
assume(result.rho, 'real');
result.C = sym(zeros(size(C)));

index = 0;
for i = 1:size(C, 1)
    for j = 1:size(C, 2)
        if (i < j) && (C(i, j) == 1)
            
            index = index + 1;
            result.map(index, :) = [i, j];
            
            result.C(i, j) = result.rho(index);
            
        end
    end
end

result.C = result.C + result.C';

result.map = result.map(1:index, :);
result.rho = result.rho(1:index);
end