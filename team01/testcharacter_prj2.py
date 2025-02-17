# This is necessary to find the main code
from asyncio import PriorityQueue
import sys

from sensed_world import SensedWorld
from world import World
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity, MonsterEntity
from colorama import Fore, Back

class TestCharacter(CharacterEntity):

    # Toggle Debug Statements:
    DEBUG = True

    # Runs when it is this Character's turn 
    def do(self, wrld):
        # put AI-behavior code HERE:

        # Important variables
        MySquare = (self.x, self.y)
        ExitSquare = self.exit_location(wrld)
        MonsterLocations = self.monster_locations(wrld)
        WallLocations = self.wall_locations(wrld)
        if(self.DEBUG):
            print("My Position: ", MySquare)
            print("Goal Position: ", ExitSquare)
            print("Monster Positions: ", MonsterLocations)
            print("Wall Positions: ", WallLocations)

        # state machine 
        state = self.state_machine(wrld)
        match state:
            case 0:
                pass
            case 1:
                pass
            case _: # default, state unaccounted for
                print("WARNING: state not accounted for, please add proper behavior")
                pass
        
    ### Helper functions ###

    # Heuristic Function for Chebychev Distance
    def chebyshev(self, x1, y1, x2, y2):
        return max(abs(x2-x1), abs(y2-y1))

    # Heuristic Function for Manhattan Distance
    def manhattan(self, x1, y1, x2, y2):
        return (abs(x1 - x2) + abs(y1 - y2))
    
    # Heuristic function for Euclidian Distance
    def euclidian(self, x1, y1, x2, y2):
        return pow(pow((x1 - x2), 2) + pow((y1 - y2), 2), 0.5)
    
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
    
    # Function to return location of all monsters on the map
    def monster_locations(self, wrld):
        s_world = SensedWorld.from_world(wrld)
        monsters = list(s_world.monsters.values())
        monsterPoses = []

        # generate list of Monster coordinates from sensed world
        for monster in monsters:
            temp = monster[0]
            monsterPoses.append((temp.x, temp.y))
        
        return monsterPoses
    
    # Function to return location of all walls on the map
    def wall_locations(self, wrld):
        walls = []
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if(wrld.wall_at(i, j)):
                    walls.append((i, j))
        return walls

    
    # Function to return exit location(standardize semantics)
    def exit_location(self, wrld):
        return wrld.exitcell
    
    # Function to reconstruct path (for A-Star)
    def trace_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            self.set_cell_color(current[0], current[1], Fore.RED + Back.GREEN)
            current = came_from[current]
        return path
    
    ### State Machine ###

    # Function for State change conditions
    def state_machine(self, wrld):
        state = 0 # default state (currently: A-Star)
        if(False):
            state = 1
        return state

    ### A-Star Algorithm ###

    # A-Star tor search for optimal path from a start to a goal, if any exists
    def A_star(self, wrld:World, start_X, start_Y, goal_X, goal_Y):
        start = (start_X, start_Y)
        goal = (goal_X, goal_Y)

        # "Tables" to record frontier, visited nodes, and heuristic values
        frontier = []
        frontier.append(start)
        explored = set()
        came_from = {start: None}
        g_count = {start: 0}
        f_count = {start: self.chebyshev(start[0], start[1], goal[0], goal[1])}

        # A-Star Loop
        while frontier:
            # Use node in frontier with smallest F value
            curr = min(frontier, key=lambda x: f_count[x])

            # Start path tracing if current node is the goal
            if curr == goal:
                path = self.trace_path(came_from, curr)
                return path  # return path (unmodified as a stack)

            # Move Current node to visited nodes
            frontier.remove(curr)
            explored.add(curr)

            # check valid neighboring nodes (Validation controlled in accompanying helper)
            for neighbor in self.neighbors(wrld, curr[0], curr[1]):
                # skip node if already visited
                if neighbor in explored:
                    continue

                # current g value calculation
                t_g_count = g_count[curr] + self.chebyshev(curr[0], curr[1], neighbor[0], neighbor[1])

                # check if neigbor is in frontier already or if better t value has been found
                if neighbor not in frontier:
                    frontier.append(neighbor)
                    # self.set_cell_color(neighbor[0], neighbor[1], Back.BLUE) # Frontier visualization
                elif t_g_count >= g_count[neighbor]:
                    continue

                # Add node to the came_from dictionary, G values, and F values for frontier search and path reconstruction
                came_from[neighbor] = curr
                g_count[neighbor] = t_g_count
                f_count[neighbor] = g_count[neighbor] + self.heuristic(neighbor[0], neighbor[1], goal[0], goal[1])

        return []  # Return an empty list if no path to the exit could be found
