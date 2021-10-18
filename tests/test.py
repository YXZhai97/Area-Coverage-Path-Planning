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

a=np.zeros((4,6,6))
a[1,3,5]=2
print(a[1])