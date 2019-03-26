function [ v05, w05 ] = lynx_velocities( thetas, thetadot )
%LYNX_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x5 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x5 matrix
%    The output has 2 parts:
%    v05 - The linear velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    w05 - The angular velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    %% YOUR CODE GOES HERE
    theta1 = thetas(1);
    theta2 = thetas(2);
    theta3 = thetas(3);
    theta4 = thetas(4);
    theta5 = thetas(5);

    A01 = compute_dh_matrix(0, -pi/2, 3, theta1);
    A12 = compute_dh_matrix(5.75, 0, 0, theta2 - pi/2);
    A23 = compute_dh_matrix(7.375, 0, 0, theta3 + pi/2);
    A34 = compute_dh_matrix(0, -pi/2, 0, theta4 - pi/2);
    A45 = compute_dh_matrix(0, 0, 4.125, theta5);

	A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;

    z0 = [0; 0; 1];
    z1 = A01(1:3, 3);
    z2 = A02(1:3, 3);
    z3 = A03(1:3, 3);
    z4 = A04(1:3, 3);
    z5 = A05(1:3, 3);

    p0 = [0; 0; 0];
    p1 = A01(1:3, 4);
    p2 = A02(1:3, 4);
    p3 = A03(1:3, 4);
    p4 = A04(1:3, 4);
    p5 = A05(1:3, 4);

    J(1:3, 1) = cross(z0, p5 - p0);
    J(4:6, 1) = z0;

    J(1:3, 2) = cross(z1, p5 - p1);
    J(4:6, 2) = z1;
    
    J(1:3, 3) = cross(z2, p5 - p2);
    J(4:6, 3) = z2;

    J(1:3, 4) = cross(z3, p5 - p3);
    J(4:6, 4) = z3;

    J(1:3, 5) = cross(z4, p5 - p4);
    J(4:6, 5) = z4;
    J
    V = J * thetadot';

    v05 = V(1:3)';
    w05 = V(4:6)';
    
end