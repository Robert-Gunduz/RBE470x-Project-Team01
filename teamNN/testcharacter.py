# This is necessary to find the main code
from asyncio import PriorityQueue
import sys

from world import World
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class TestCharacter(CharacterEntity):

    # Runs when it is this Character's turn 
    def do(self, wrld):
        # put AI-behavior code HERE:
        (dx, dy) = self.next_move(wrld) # A-Star finds next move
        self.move(dx - self.x, dy - self.y)
    
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

    # Function to check for valid neighboring Cells, returns list of coordinates (modified from 'look_for_empty_cell')
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
                            cells.append((x + dx, y + dy))
        # All done
        return cells

    # Distance Helper Function (choose one provided by comment/uncomment)
    def heuristic(self, x1, y1, x2, y2):
        #return max(abs(x2-x1), abs(y2-y1)) # Chebyshev
        #return (abs(x1 - x2) + abs(y1 - y2)) # Manhattan
        return pow(pow((x1 - x2), 2) + pow((y1 - y2), 2), 0.5) # Pythagorean

    # reconstructs path in stack form
    def trace_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path

    # returns the next move coordinates suggested by A-Star
    def next_move(self, wrld:World):
        start = (self.x, self.y)
        Exit_x, Exit_y = 0, 0

        #Find the goal in the World space
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.exit_at(i, j):
                    Exit_x = i
                    Exit_y = j
                    break

        goal = (Exit_x, Exit_y)

        # "Tables" to record frontier, visited nodes, and heuristic values
        frontier = []
        frontier.append(start)
        explored = set()
        came_from = {start: None}
        g_count = {start: 0}
        f_count = {start: self.heuristic(self.x, self.y, Exit_x, Exit_y)}

        # A-Star Loop
        while frontier:
            # Use node in frontier with smallest F value
            curr = min(frontier, key=lambda x: f_count[x])

            # Start path tracing if current node is the goal
            if curr == goal:
                path = self.trace_path(came_from, curr)
                path.pop() # Pop start node (currently occupied by character)
                return path.pop() # return the next node (suggested next move from path)
            
            # Move Current node to visited nodes
            frontier.remove(curr)
            explored.add(curr)

            # check valid neighboring nodes (Validation controlled in accompanying helper)
            for neighbor in self.neighbors(wrld, curr[0], curr[1]):
                # skip node if already visited
                if neighbor in explored:
                    continue
                
                # current g value calculation
                t_g_count = g_count[curr] + self.heuristic(curr[0], curr[1], neighbor[0], neighbor[1])

                # check if neigbor is in frontier already or if better t value has been found
                if neighbor not in frontier:
                    frontier.append(neighbor)
                elif t_g_count >= g_count[neighbor]:
                    continue
                
                # Add node to the came_from dictionary, G values, and F values for frontier search and path reconstruction
                came_from[neighbor] = curr
                g_count[neighbor] = t_g_count
                f_count[neighbor] = g_count[neighbor] + self.heuristic(neighbor[0], neighbor[1], goal[0], goal[1])
        
        return None # Return Nonetype if no valid path has been found to the End Position