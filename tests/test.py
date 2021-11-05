# import sample
#
#
# agent1 = sample.AlphaAgent()
# print(agent1.id)
# print(sample.AlphaAgent.number_of_agent)
# print(agent1.number_of_agent)
#
# agent2 = sample.AlphaAgent()
# print(agent2.id)
# print(sample.AlphaAgent.number_of_agent)
# print(agent2.number_of_agent)
#
# # test floodFill
# image = [
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
#     [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
#     [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
#     [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
#     [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
#     [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 1, 1, 0, 0, 1, 1],
#     [0, 0, 0, 1, 0, 0, 1, 0, 1, 1],
#     [0, 0, 0, 1, 0, 0, 1, 1, 1, 1],
#     [0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
# ]
#
# print(sample.floodFill(image, 3, 3, 1))

import numpy as np
import matplotlib.pyplot as plt
# a=np.zeros((4,6,6))
# a[1,3,5]=2
# print(a[1])
# b=np.zeros((10,4))
# b[0,:2]=[1,2]
# print(b)
# n=np.array(b)[0,:2]
# print(n)

neighbour=[[1,2,3,4],[5,6,7,8]]
for n in neighbour:
    print(n)
print(len(neighbour))

testM=np.array([[1,2,3,4],
       [3,4,5,6]])

print(testM[-1])
newrow=np.zeros(4)
testM=np.append(testM,[newrow],axis=0)
print(testM)

a=np.array([1,2,3])
print(a*0.1)

print(np.zeros(4))
for m in testM:
    m=np.append(m,1)
    print(m)

a=[1,2,3,4,5]
b=[6,7,8,9,10]

for i,j in zip(a,b):
    print(i+j)

print(5//2)

fig1=plt.figure('Figure 1 with subplots', figsize=(10,4))
subfig1=fig1.add_subplot(121)
subfig1.plot([1,2,3,4],[5,6,7,8])
subfig2=fig1.add_subplot(122)
subfig2.scatter([5,6,7,8],[1,2,3,4])

fig2=plt.figure('Figure 2 with subplots', figsize=(10,4))
subfig1=fig2.add_subplot(121)
subfig1.scatter([1,2,3,4],[5,6,7,8])

grid=np.zeros((4,4))
grid[:,1]=1
print(grid)

a_k=np.array([1,2])
a=np.array([1,1])
b=np.array([1,4])
c=[1,2,3]
c_n=np.array(b)
print(c)

norm=np.linalg.norm(a-b)
print(norm.shape)
print(a_k.shape)
print(a_k/norm)

neighbour=[]
n1=[1,2]
n2=[3,4]
neighbour.append(n1)
neighbour.append(n2)
print(neighbour)
for n in neighbour:
    n.append(0)
    print(n)

neighbour=[[1,2,3,4]]
print(len(neighbour))

nn=np.zeros(0)
print(nn)

nnn=np.array([1,2,3])
print(2*nnn)

a=np.array([[2,2,3,4],[5,6,7,8]])
b=[]
for aa in a:
    aa = np.append(aa, [1, 2])
    print(aa)
    b.append(aa)

print(a)
print(b)
print(len(b))
print(b[1])

fig1 = plt.figure('Figure1',figsize = (6,4))
fig1.add_subplot(221)
fig1.plot([1,2,3,4],[5,6,7,8])
fig1.add_subplot(221)
fig1.plot([1,2,3,4],[5,6,7,8])
