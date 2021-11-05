import numpy as np

def velocity_projection(p,p_beta,v):

    p=np.array(p)
    print(p)
    p_beta=np.array((p_beta))
    print(p_beta)
    I= np.identity(2)
    print(I)
    a_k=(p-p_beta)/np.linalg.norm(p-p_beta)
    print(a_k)
    proj_matrix=I-np.outer(a_k,a_k.T)
    print(proj_matrix)
    v_proj=np.dot(v,proj_matrix)

    return v_proj

v_p=velocity_projection([3,5],[4,7],[3,3])
print(v_p)
# a=np.array([1,2,3])
# p=np.outer(a,a.T)
# print(np.outer(a,a.T))
# print(np.dot(p,a))
#
pp=np.array([[1,2,3],[4,6,8]])
v=np.array([1,2])
for i in range(len(pp)):
    np.append(pp[i], 1)

print(pp)


