function [ T01, T12 ] = sawyer_fk( a1, d1, d2, q1, q2 )
    syms pi
    T01 = compute_dh_matrix_sym(a1, -pi/2, d1, q1);
    T12 = compute_dh_matrix_sym(0, -pi/2, d2, q2);
end
    