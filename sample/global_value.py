import math

robotList=[]
d_alpha=12
d_beta=6
x_bound=300
y_bound=300
v_bound=2
env_map=[]
epsilon=0.08
h_alpha=0.2
h_beta=0.9
c1_alpha=60
c2_alpha=2*math.sqrt(c1_alpha)
c1_beta=60
c2_beta=2*math.sqrt(c1_beta)

# grid map property
x_bound=300
y_bound=300
grid_length=3
x_n=int(x_bound/grid_length)
y_n=int(y_bound/grid_length)

# value in benefit function
k1=0.04
k2=0.01
rohgamma=0.2

# simulation
T=30 # total simulation time
step_size=0.01 #
Iteration=int(T/step_size) # total iteration number



