from sample.floodFill import floodFill
from sample.robot import AlphaAgent

agent1 = AlphaAgent()
print(agent1.id)
print(AlphaAgent.number_of_agent)
print(agent1.number_of_agent)

agent2 = AlphaAgent()
print(agent2.id)
print(AlphaAgent.number_of_agent)
print(agent2.number_of_agent)

# test floodFill
image = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 0, 0, 1, 1],
    [0, 0, 0, 1, 0, 0, 1, 0, 1, 1],
    [0, 0, 0, 1, 0, 0, 1, 1, 1, 1],
    [0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
]

print(floodFill(image, 3, 3, 1))