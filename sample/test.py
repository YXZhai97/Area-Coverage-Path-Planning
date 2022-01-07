import numpy as np
import global_value as gv
import methods as m

q_beta=np.array([1,2])
q_i=np.array([3,3])
u_beta= gv.c1_beta * m.phi_beta(m.sigma_norm(q_beta - q_i)) * m.norm_direction(q_beta, q_i)
print(u_beta)