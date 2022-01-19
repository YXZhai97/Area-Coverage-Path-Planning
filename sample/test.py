import matplotlib.pyplot as plt
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

end_points=[[1,2],[3,4],[1,1]]
for i in end_points:
    print(i[1])


state=np.zeros((5,4))
state_i=state[0:3]
merge=[]
merge.append(state_i)
state_j=state[2:4]
merge.append(state_j)
print(state[0:2])
print(merge)
mermer=[]
mermer.append(merge)
mermer.append(merge)
print(mermer)

color=['a','b','c']
mylist=[1,2,3]
for i, c in zip(mylist, color):
    print(c)
    print(i)

figure = plt.figure('robot path at different time ', figsize=(15, 10))

r=np.random.uniform(0.9, 1.2) * 3
print(r)