function [ v06, w06 ] = puma_velocities( thetas, thetadot )
%PUMA_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x6 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x6 matrix
%    The output has 2 parts:
%    v06 - The linear velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    w06 - The angular velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    %% YOUR CODE GOES HERE
    theta1 = thetas(1);
    theta2 = thetas(2);
    theta3 = thetas(3);
    theta4 = thetas(4);
    theta5 = thetas(5);
    theta6 = thetas(6);

    a = 13.0;
    b = 2.5;
    c = 8.0;
    d = 2.5;
    e = 8.0;
    f = 2.5;

    A01 = compute_dh_matrix(0, pi/2, a, theta1)
    A12 = compute_dh_matrix(c, 0, -b, theta2);
    A23 = compute_dh_matrix(0, -pi/2, -d, theta3);
    A34 = compute_dh_matrix(0, pi/2, e, theta4);
    A45 = compute_dh_matrix(0, -pi/2, 0, theta5);
    A56 = compute_dh_matrix(0, 0, f, theta6);

    A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;
    A06 = A05 * A56;

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
    p6 = A06(1:3, 4);

    J(1:3, 1) = cross(z0, p6 - p0);
    J(4:6, 1) = z0;

    J(1:3, 2) = cross(z1, p6 - p1);
    J(4:6, 2) = z1;
    
    J(1:3, 3) = cross(z2, p6 - p2);
    J(4:6, 3) = z2;

    J(1:3, 4) = cross(z3, p6 - p3);
    J(4:6, 4) = z3;

    J(1:3, 5) = cross(z4, p6 - p4);
    J(4:6, 5) = z4;

    J(1:3, 6) = cross(z5, p6 - p5);
    J(4:6, 6) = z5;
    
    V = J * thetadot';
    
    
    v06 = V(1:3)';
    w06 = V(4:6)';
    
end