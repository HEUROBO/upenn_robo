function [ vp1 vp2 ] = velocity( a1, d1, d2, q1, q2, c1, c2, c3, qd1, qd2)
    syms pi
    T01 = compute_dh_matrix_sym(a1, -pi/2, d1, q1);
    T12 = compute_dh_matrix_sym(0, -pi/2, d2, q2);
    T02 = T01 * T12;
    p11 = [0; 0; c1; 1]; % position of p1 in frame 1
    p10 = T01 * p11;
    p10 = p10(1:3);
    j1 = [0; 0; 1];
    vp1 = cross(j1, p10) * qd1;

    % slide 16 and 20 week7 part 1 robo1x
    o1 = T01(1:3, 4);
    p22 = [0; -c3; c2; 1];
    p20 = T02 * p22;
    p20 = p20(1:3)
    z1 = T01(1:3, 3);
    j2 = cross(z1, p20 - o1);
    z0 = [0; 0; 1];
    j1 = cross(z0, p20);
    vp2 = j1 * qd1 + j2 * qd2;
end