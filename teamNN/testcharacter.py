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
        (dx, dy) = self.next_move(wrld)
        print("Goal Found")
        self.move(dx, dy)
    
    # Find movement difficulty of terrain given in grid form
    def MapCheck(self, wrld:World):
        W_width = wrld.width()
        W_height = wrld.height()
        Exit_x = 0
        Exit_y = 0
        Hx = [W_width][W_height]
        # Find the Exit Position on the Grid
        for i in range(W_width):
            for j in range(W_height):
                if wrld.exit_at(i, j):
                    Exit_x = i
                    Exit_y = j
                    break
        # Calculate Movement difficulty (Wall Presence + Manhattan dist)
        for i in range(W_width):
            for j in range(W_width):
                # Manhattan Distance
                Hx[i][j] = abs(i - Exit_x) + abs(j - Exit_y)
                # Check for Wall
                if wrld.wall_at(i, j):
                    Hx[i][j] += 50
        return Hx

    def neighbors(self, wrld, x, y):
        # List of empty cells
        cells = []
        # Go through neighboring cells
        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((x + dx >= 0) and (x + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((y + dy >= 0) and (y + dy < wrld.height())):
                        # Is this cell safe?
                        if(wrld.exit_at(x + dx, y + dy) or
                           wrld.empty_at(x + dx, y + dy)):
                            # Yes
                            cells.append((dx, dy))
        # All done
        return cells

    # Manhattan Distance Helper Function
    def heuristic(self, x1, y1, x2, y2):
        return (abs(x1 - x2) + abs(y1 - y2))

    def next_move(self, wrld:World):
        start = (self.x, self.y)
        Exit_x = 0
        Exit_y = 0
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.exit_at(i, j):
                    Exit_x = i
                    Exit_y = j
                    break

        frontier = [start]
        explored = []
        came_from = {start: None}
        g_count = {start: 0}
        f_count = {start: self.heuristic(self.x, self.y, Exit_x, Exit_y)}

        while frontier:
            curr = min(frontier, key=lambda x: f_count[x])

            if curr == (Exit_x, Exit_y):
                print("Goal Found")
                traceBack = curr
                while came_from[traceBack] != start:
                    if traceBack is None or traceBack not in came_from:
                        print("No valid path to start")
                        return None
                    traceBack = came_from[traceBack]
                return (traceBack[0], traceBack[1])
            
            frontier.remove(curr)
            explored.append(curr)

            for neighbor in self.neighbors(wrld, curr[0], curr[1]):
                if neighbor in explored:
                    continue

                t_g_count = g_count[curr] + self.heuristic(curr[0], curr[1], neighbor[0], neighbor[1])

                if neighbor not in frontier:
                    frontier.append(neighbor)
                elif t_g_count >= g_count[neighbor]:
                    continue

                came_from[neighbor] = curr
                g_count[neighbor] = t_g_count
                f_count[neighbor] = g_count[neighbor] + self.heuristic(neighbor[0], neighbor[1], Exit_x, Exit_y)
        
        #return (1, 1)

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