import numpy as np

state=np.array([[1,2,3,4],
       [3,4,5,6],
       [1,1,1,1]])

state_part=state[:2,1]
print(state_part)