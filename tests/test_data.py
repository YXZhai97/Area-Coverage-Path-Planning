import numpy as np

state=np.array([[1,2,3,4],
       [3,4,5,6],
       [1,1,1,1]])

state_part=state[:2,1]
print(state_part)


target=np.zeros((2,2))
target[1]=[1,2]
print(target)
x=np.random.uniform(0.1, 0.9) * 10
print(x)