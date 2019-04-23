function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
current_pos = start_coords;
route = zeros(max_its, 2);
actual_it = -1;
route(1, :) = start_coords;
for n=2:max_its + 1
    dx = gx(round(current_pos(2)), round(current_pos(1)));
    dy = gy(round(current_pos(2)), round(current_pos(1)));
    grad = [dx, dy];
    grad = grad / norm(grad);
    current_pos = current_pos + grad; 
    route(n, :) = current_pos;
    d = sqrt((current_pos(1) - end_coords(1))^2 + (current_pos(2) - end_coords(2))^2);
    if d < 2
        actual_it = n;
        break
    end
    actual_it = n;
end
route = route(1:actual_it, :);
size(route)
% *******************************************************************
end
