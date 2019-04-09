function [pos, R] = puma_fk(theta1, theta2, theta3, theta4, theta5, theta6)
    %PUMA_FK The input to the function will be the joint angles of the robot in radians.
    %    The output must contain end effector position of the robot arm and the rotation matrix representing the rotation from frame
    %    6 to frame 0, as specified in the question.
    
        %% Your code goes here
        a = 13.0;
        b = 2.5;
        c = 8.0;
        d = 2.5;
        e = 8.0;
        f = 2.5;

        A01 = compute_dh_matrix(0, pi/2, a, theta1);
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
        
        pos = A06(1:3, 4)';
        R = A06(1:3, 1:3);
        
    end
    
function A = compute_dh_matrix(r, alpha, d, theta)

    %% It might be helpful to copy over this function from Week 5 Lab
    A = eye(4);
%     A = sym(A);
    A(1,1) = cos(theta);
    A(2,1) = sin(theta);
    A(3,1) = 0;
    A(4,1) = 0;
    A(1,2) = -sin(theta) * cos(alpha);
    A(2,2) = cos(theta) * cos(alpha);
    A(3,2) = sin(alpha);
    A(4,2) = 0;
    A(1,3) = sin(theta) * sin(alpha);
    A(2,3) = -cos(theta) * sin(alpha);
    A(3,3) = cos(alpha);
    A(4,3) = 0;
    A(1,4) = r * cos(theta);
    A(2,4) = r * sin(theta);
    A(3,4) = d;
    A(4,4) = 1;
end
    