function [pos, R] = puma_fk_sym(theta1, theta2, theta3, theta4, theta5, theta6)
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

        A01 = compute_dh_matrix_sym(0, sym(pi)/2, a, theta1);
        A12 = compute_dh_matrix_sym(c, 0, -b, theta2);
        A23 = compute_dh_matrix_sym(0, -sym(pi)/2, -d, theta3);
        A34 = compute_dh_matrix_sym(0, sym(pi)/2, e, theta4);
        A45 = compute_dh_matrix_sym(0, -sym(pi)/2, 0, theta5);
        A56 = compute_dh_matrix_sym(0, 0, f, theta6);
        A34 * A45 * A56

        A02 = A01 * A12;
        A03 = A02 * A23;
        A04 = A03 * A34;
        A05 = A04 * A45;
        A06 = A05 * A56;
        
        pos = A03(1:3, 4)';
        R = A03(1:3, 1:3);
        
    end
    

    