function A = test_RPR_ik(theta1, d2, theta3)
	[pos, R] = RPR_fk(theta1, d2, theta3)
	ik_sol = RPR_ik( pos(4, 1), pos(4, 2), pos(4, 3), R )
	[pos_ik, R_ik] = RPR_fk(ik_sol(1),ik_sol(2),ik_sol(3))
	assert(sum(sum(abs(pos - pos_ik))) < 1e-3, 'pos failed!')
	assert(sum(sum(abs(R - R_ik))) < 1e-3, 'R failed!')
	disp('All test passed!')
end