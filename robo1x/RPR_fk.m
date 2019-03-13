function [ pos, R ] = RPR_fk( theta1, d2, theta3 )
%RPR_FK Write your code here. The input to the function will be the joint
%    angles of the robot in radians, and the extension of the prismatic joint in inches.
%    The output includes: 
%    1) The position of the end effector and the position of 
%    each of the joints of the robot, as explained in the question.
%    2) The rotation matrix R_03, as explained in the question.

    %% YOUR CODE GOES HERE
    
    pos = zeros(4, 3);
    R = eye(3);
    R01 = compute_dh_matrix(0, -3*pi/4, 10, theta1);
    R12 = compute_dh_matrix(0, -pi/2, d2, -pi/2);
    R23 = compute_dh_matrix(5, 0, 0, theta3 - pi/4) * compute_dh_matrix(0, pi/2, 0, pi/2) * compute_dh_matrix(0, 0, 0, pi/2);
    R = R01 * R12 * R23;
    % R = R(1:3, 1:3);
    pos(1,:) = [0 0 0];
    temp = R01 * [0 0 0 1]';
    pos(2,:) = temp(1:3);
    temp  = R01 * R12 * [0 0 0 1]';
    pos(3,:) = temp(1:3);
    temp = R01 * R12 * R23 *  [0 0 0 1]';
    pos(4,:) = temp(1:3);
    
end
