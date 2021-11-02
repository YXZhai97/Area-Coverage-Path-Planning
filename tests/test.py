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
