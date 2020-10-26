%Gives you boolean matrix of the form:
% [False True  True  True]
% [False False True  True]
% [False False False True]
% [False False False False]
%
% of appropriate dimentions
%
function P = helper_get_upper_right_courner_incides(n)

P = false(n, n);

for i = 1:n
    for j = 1:n
        if i < j
            P(i, j) = true;
        end
    end
end
end