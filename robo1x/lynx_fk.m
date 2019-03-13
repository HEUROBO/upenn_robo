function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.
%    The output must contain 10 positions of various points along the robot arm as specified
%    in the question.

    %% YOUR CODE GOES HERE
    
    pos = zeros(10, 3);
%     A01 = compute_dh_matrix(0, pi/2, 3, -theta1);
%     A12 = compute_dh_matrix(5.75, 0, 0, -theta2 + pi/2);
%     A23 = compute_dh_matrix(7.375, 0, 0, -theta3 - pi/2);
%     A34 = compute_dh_matrix(0, pi/2, 0, -theta4 + pi/2);
%     A45 = compute_dh_matrix(0, 0, 4.125, -theta5);
    A01 = compute_dh_matrix(0, -pi/2, 3, theta1);
    A12 = compute_dh_matrix(5.75, 0, 0, theta2 - pi/2);
    A23 = compute_dh_matrix(7.375, 0, 0, theta3 + pi/2);
    A34 = compute_dh_matrix(0, -pi/2, 0, theta4 - pi/2);
    A45 = compute_dh_matrix(0, 0, 4.125, theta5);


    A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;
    
    pos(1,:) = [0 0 0];
    temp = A01 * [0 0 0 1]';
    pos(2,:) = temp(1:3);
    temp = A02 * [0 0 0 1]';
    pos(3,:) = temp(1:3);
    temp = A03 * [0 0 0 1]';
    pos(4,:) = temp(1:3);
    temp = A04 * [0 0 0 1]';
    pos(5,:) = temp(1:3);
    
    e = 1.125;
    temp = A05 * [0 0 -e 1]';
    pos(6,:) = temp(1:3);
    temp = A05 * [g/2 0 -e 1]';
    pos(7,:) = temp(1:3);
    temp = A05 * [-g/2 0 -e 1]';
    pos(8,:) = temp(1:3);
    temp = A05 * [g/2 0 0 1]';
    pos(9,:) = temp(1:3);
    temp = A05 * [-g/2 0 0 1]';
    pos(10,:) = temp(1:3);
    
end