# This is necessary to find the main code
from asyncio import PriorityQueue
import sys

from world import World
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here
        pass
    
    # Find movement difficulty of terrain given
    def MapCheck(self, wrld:World):
        W_width = wrld.width()
        W_height = wrld.height()
        Hx = [W_width][W_height]
        Epos = wrld.exit_at()
        for i in range(W_width):
            for j in range(W_height):
                pass
                


    # Move this to a separate file later, base code from: https://www.redblobgames.com/pathfinding/a-star/introduction.html
    def A_Star(self, wrld:World):
        frontier = PriorityQueue()

        frontier.put(start, 0)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            
            for next in graph.neighbors(current):
                new_cost = cost_so_far[current] + graph.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current