import numpy as np
import global_value as gv
import methods as m

q_beta=np.array([1,2])
q_i=np.array([3,3])
u_beta= gv.c1_beta * m.phi_beta(m.sigma_norm(q_beta - q_i)) * m.norm_direction(q_beta, q_i)
print(u_beta)

time_steps = np.linspace(0, gv.step_size*10, 11)
print(time_steps)

percent=[1,2,3,4,5,6,7]
print(percent[:3])

target=np.array([[1,2],[3,4],[5,6]])
print(target[:2,0])

target=[1,2,3,4,5,6]
print(target[1:])

end_points=[[1,2],[3,4]]
print(end_points[0])