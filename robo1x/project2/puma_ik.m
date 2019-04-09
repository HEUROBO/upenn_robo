function [ ik_sol ] = puma_ik( x, y, z, R )
    %PUMA_IK Write your code here. The input to the function will be the position of
    %    the end effector (in inches) in the world frame, and the 
    %    Rotation matrix R_60 as described in the question.
    %    The output must be the joint angles of the robot to achieve 
    %    the desired end effector position and orientation.
    
    %% YOUR CODE GOES HERE
    a = 13.0;
    b = 2.5;
    c = 8.0;
    d = 2.5;
    e = 8.0;
    f = 2.5;

    z6 = R(:, 3);
    p_center = [x; y; z] - f * z6;
    p_center = [p_center; 1];
    x_center = p_center(1);
    y_center = p_center(2);

    % compute theta1
    y_center_1 = b + d; % y coordinanate of p_center in frame 1
    theta1 = asin(-y_center_1/(sqrt(x_center * x_center + y_center * y_center))) + atan2(y_center, x_center);
    theta1 = real(theta1);
    % compute theta2
    A01 = compute_dh_matrix(0, pi/2, a, theta1);
    p_center_1 = real(inv(A01)) * p_center;
    x_center_1 = p_center_1(1);
    y_center_1 = p_center_1(2);
    theta2 = asin((x_center_1^2 + y_center_1^2 + c^2 - e^2) / (2 * c * sqrt(x_center_1^2 + y_center_1^2))) - atan2(x_center_1, y_center_1);
    % compute theta3
    theta3 = pi - theta2 + atan2(c * sin(theta2) - y_center_1 , c * cos(theta2) - x_center_1) - pi / 2;
    % compute theta4, theta5, theta6
    A12 = compute_dh_matrix(c, 0, -b, theta2);
    A23 = compute_dh_matrix(0, -pi/2, -d, theta3);
    A03 = A01 * A12 * A23;
    A06 = zeros(4, 4);
    A06(1:3, 1:3) = R;
    A06(:, 4) = [x; y; z; 1];
    A36 = inv(A03) * A06;
    theta4 = atan2(-A36(2, 3), -A36(1, 3));
    theta5 = acos(A36(3, 3));
    theta6 = atan2(-A36(3, 2), A36(3, 1));
    ik_sol = [theta1, theta2, theta3, theta4, theta5, theta6]
end