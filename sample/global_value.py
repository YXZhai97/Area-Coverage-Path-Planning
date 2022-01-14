import math

# robot parameters
# rs=4 suitable for 50x50 grid map rs=3 suitable for 30x30 map
rs=4
rc=6
r_tan=rs/5
target_distance=rs
dimension=2
robotList=[] # a list of robot
d_alpha=12
d_beta=6 # the min distance between beta and alpha
epsilon=0.08
h_alpha=0.2
h_beta=0.9
c1_alpha=60
c2_alpha=2*math.sqrt(c1_alpha)
c1_beta=60
c2_beta=2*math.sqrt(c1_beta)
c1_gamma=30
c2_gamma=2*math.sqrt(c1_gamma)

# robot map information
grid_map=[]

# environment parameters
x_bound=30
y_bound=30
v_bound=2
grid_length=1
x_n=int(x_bound/grid_length) # number of grid in x direction
y_n=int(y_bound/grid_length) # number of grid in y direction
env_map=[]


# value in benefit function
k1=0.04
k2=0.01
rohgamma=0.2
# simulation parameters
robot_number=3
T=80 # total simulation time
step_size=0.1 # size of each step
Iteration=int(T/step_size) # total iteration number
rate=1






