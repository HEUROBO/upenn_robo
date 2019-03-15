function [ ik_sol ] = RPR_ik( x, y, z, R )
%RPR_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_30 as described in the question.
%    The output must be the joint angles and extensions of the robot to achieve 
%    the end effector position and orientation.

    %% YOUR CODE GOES HERE
    
    
    ik_sol = ones(1, 3);
    d2 = -(z - 5 * R(3, 3) - 10) * sqrt(2);
    % theta1 = atan2(-x + 5 * R(1, 3), y - 5 * R(2, 3));
    theta1 = atan2(R(1, 2), R(1, 1))
    theta3 = atan2(R(3, 2) + R(3, 3), R(3, 3) - R(3, 2)) + pi/4;
    ik_sol = [theta1, d2, theta3];

end