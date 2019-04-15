function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
    for j = 1:2
        if j == 1
            P = P1;
            other_P = P2;
        else
            P = P2;
            other_P = P1;
        end
        for i = 0:2
            rem(i, 3) + 1
            if is_opposite_side(P(rem(i, 3) + 1, :), P(rem(i + 1, 3) + 1, :), P(rem(i + 2, 3) + 1, :), other_P(1, :)) ...
            && is_opposite_side(P(rem(i, 3) + 1, :), P(rem(i + 1, 3) + 1, :), P(rem(i + 2, 3) + 1, :), other_P(2, :)) ...
            && is_opposite_side(P(rem(i, 3) + 1, :), P(rem(i + 1, 3) + 1, :), P(rem(i + 2, 3) + 1, :), other_P(3, :))
                flag = false;
                return
            end
        end
    end
    flag = true;
% *******************************************************************
end

function flag = is_opposite_side(point_line_1, point_line_2, point_1, point_2)
    v =  point_line_2 - point_line_1;
    v1 = point_1 - point_line_1;
    v2 = point_2 - point_line_1;
    flag = (v(1) * v1(2) - v(2) * v1(1)) * (v(1) * v2(2) - v(2) * v2(1)) < 0;
end