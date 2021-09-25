"""
Define alpha agent
The property of the alpha agent is defined
The method of the alpha agent is defined

"""
from typing import List



class AlphaAgent:
    number_of_agent = 0  # the number of agent defined

    def __init__(self):
        self.id = self.number_of_agent
        self.rs = 10
        self.rc = 60
        self.map = []
        self.initial_state = []
        self.state = []
        self.neighbour = []
        AlphaAgent.add_agent()  # call the class method when initialize the object

    @classmethod
    def add_agent(cls):
        cls.number_of_agent += 1


class Environment:
    def __init__(self):

        pass

    def add_obstacle(self):
        pass


class Obstacle:

    def __init__(self):
        pass






# Test
agent1 = AlphaAgent()
print(agent1.id)
print(AlphaAgent.number_of_agent)
print(agent1.number_of_agent)

agent2 = AlphaAgent()
print(agent2.id)
print(AlphaAgent.number_of_agent)
print(agent2.number_of_agent)
