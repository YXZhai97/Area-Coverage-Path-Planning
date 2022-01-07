import numpy as np
import global_value as gv
state=np.array([[1,2,3,4],
       [3,4,5,6],
       [1,1,1,1]])

state_part=state[:2,1]
print(state_part)

q_beta=np.array([1,2])
q_i=np.array([3,3])
u_beta= gv.c1_beta * m.phi_beta(m.sigma_norm(q_beta - q_i)) * m.norm_direction(q_beta, q_i)