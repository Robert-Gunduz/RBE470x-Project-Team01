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

    # Runs when it is this Character's turn 
    def do(self, wrld):
        # put AI-behavior code HERE:
        state = 0
        self.clearMap(wrld)
        state = self.stateShift(wrld) # Check for State change
        self.senseWorld(wrld)
        print("monsterNear: ", self.monsterNear(wrld, self.x, self.y, 2))
        print("State: ", state)
        match state:
            case 0: # Move using A star as normal
                (dx, dy, _) = self.next_move(wrld, self.x, self.y) # A-Star finds next move
                print("Move direction: ", (dx, dy))
                self.move(dx - self.x, dy - self.y)
                pass
            case 1: # Monster is nearby, move to avoid monster
                # TODO: Add behavior to run away from monster\
                #(dx, dy) = self.next_move(wrld) # A-Star finds next move
                (x, y) = self.avoid_monster(wrld)
                print("Move direction: ", (x, y))
                self.move(x - self.x, y - self.y)
                pass
            case _: # default, state unaccounted for
                print("WARNING: state not accounted for, please add proper behavior")
                pass
        

    # Function for State change conditions
    def stateShift(self, wrld):
        state = 0 # default state (currently: A-Star)
        if(self.monsterNear(wrld, self.x, self.y, 3)[0] and not self.book_it(wrld, self.monsterLocs(wrld))):
            state = 1 # Avoid monster state
        return state
    
    def clearMap(self, wrld):
        for y in range(wrld.height()):
            for x in range(wrld.width()):
                self.set_cell_color(x, y, Fore.BLACK + Back.BLACK)
                pass
        
            

    def senseWorld(self, wrld:World):
        s_world = SensedWorld.from_world(wrld)
        self.W_width = s_world.width()
        self.W_height = s_world.height()
        self.W_exit = s_world.exitcell
        self.monsters = list(s_world.monsters.values())
        if (len(self.monsters)):
            monster:MonsterEntity = self.monsters.pop()[0]
            monsterPoses = (monster.x, monster.y) # change this to lambda of monster poses to get list of monster poses
        # print("Width: ", self.W_width, "\nHeight: ", self.W_height, "\nExit: ", self.W_exit, "Monster Pos: ", monsterPoses)

        # do some sensing for heuristics?
        for y in range(self.W_height):
            for x in range(self.W_width):
                # if(wrld.exitcell())
                pass

    # Function to determine if a monster is nearby, returns true if monster is within threshold
    def monsterNear(self, wrld, x, y, threshold):
        current_pos = (x, y)
        s_world = SensedWorld.from_world(wrld)
        monsters = list(s_world.monsters.values())
        monsterPoses = []

        # generate list of Monster coordinates from sensed world
        for monster in monsters:
            temp = monster[0]
            monsterPoses.append((temp.x, temp.y))

        # run check for if the monster is nearby
        for monster in monsterPoses:
            monsterDist = self.heuristic(current_pos[0], current_pos[1], monster[0], monster[1])
            if (monsterDist <= threshold):

                return (True, monsterDist) # monster is near the character
        return (False, -1) # monster is not near the character
    
    def monsterLocs(self, wrld):
        s_world = SensedWorld.from_world(wrld)
        monsters = list(s_world.monsters.values())
        monsterPoses = []

        # generate list of Monster coordinates from sensed world
        for monster in monsters:
            temp = monster[0]
            monsterPoses.append((temp.x, temp.y))
        
        return monsterPoses


    def monsterHeuristic(self, wrld, lookahead=1):
        s_world = SensedWorld.from_world(wrld)
        monsters = list(s_world.monsters.values())
        monsterPoses = []
        monsterHeuristic = dict() # Coord(x,y), heuristic
        
        # generate list of Monster coordinates from sensed world
        for monster in monsters:
            temp = monster[0]
            monsterPoses.append((temp.x, temp.y))
        pass

        for monsterPos in monsterPoses:
            poses = []
            visited = []
            poses.append(monsterPos)
            currentLookahead = 1
            while len(poses):
                pose = poses.pop()
                for neighbor in self.neighbors(s_world, pose[0], pose[1]):
                    if neighbor not in visited:
                        if (currentLookahead < lookahead):
                            poses.append(neighbor)
                        visited.append(neighbor)
                    monsterHeuristic[neighbor] =  10000 # some set number
                    self.set_cell_color(neighbor[0], neighbor[1], Back.RED)
                currentLookahead += 1
                
        return monsterHeuristic
        

        

   
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
        return max(abs(x2-x1), abs(y2-y1)) # Chebyshev
        #return (abs(x1 - x2) + abs(y1 - y2)) # Manhattan
        #return pow(pow((x1 - x2), 2) + pow((y1 - y2), 2), 0.5) # Pythagorean

    # reconstructs path in stack form
    def trace_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            self.set_cell_color(current[0], current[1], Fore.RED+Back.GREEN)
            current = came_from[current]
        return path
    
    def avoid_monster(self, wrld):
        monsterGain = 1
        exitGain = 2

        s_world = SensedWorld.from_world(wrld)
        self.W_exit = s_world.exitcell
        monsterLocs = self.monsterLocs(wrld)
        neighbors = self.neighbors(wrld, self.x, self.y)
        moveCandidates = {}

        for neighbor in neighbors:
            score = 0
            for monsterLoc in monsterLocs:
                score += monsterGain * 1/(self.heuristic(neighbor[0], neighbor[1], monsterLoc[0], monsterLoc[1]))
            pathlen = self.min_moves(wrld, neighbor[0], neighbor[1])

            score -= exitGain * 1/pathlen # * self.heuristic(neighbor[0], neighbor[1], self.W_exit[0], self.W_exit[1])
            moveCandidates[neighbor] = score

        return min(neighbors, key=lambda x: moveCandidates[x])
    
    def book_it(self, wrld, monsters):
        s_world = SensedWorld.from_world(wrld)
        exit = s_world.exitcell
        immediateTreat = (1000000,1000000)
        for monster in monsters:
            if self.heuristic(self.x, self.y, monster[0], monster[1]) < self.heuristic(self.x, self.y, immediateTreat[0], immediateTreat[1]):
                immediateTreat = (monster[0], monster[1])
        if self.heuristic(self.x, self.y, exit[0], exit[1]) < self.heuristic(exit[0], exit[1], immediateTreat[0], immediateTreat[1]):
            return True
        else:
            return False




    def min_moves(self, wrld, x, y):
        start = (x, y)
        goal = wrld.exitcell

        # "Tables" to record frontier, visited nodes, and heuristic values
        frontier = []
        frontier.append(start)
        explored = set()
        came_from = {start: None}
        g_count = {start: 0}
        f_count = {start: self.heuristic(self.x, self.y, goal[0], goal[1])}

        # A-Star Loop
        while frontier:
            # Use node in frontier with smallest F value
            curr = min(frontier, key=lambda x: f_count[x])

            # Start path tracing if current node is the goal
            if curr == goal:
                self.monsterHeuristic(wrld, 2)
                path = self.trace_path(came_from, curr)
                pathLen = len(path)
                return pathLen # return the next node (suggested next move from path)
            
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
                    self.set_cell_color(neighbor[0], neighbor[1], Back.BLUE)
                elif t_g_count >= g_count[neighbor]:
                    continue
                
                # Add node to the came_from dictionary, G values, and F values for frontier search and path reconstruction
                came_from[neighbor] = curr
                g_count[neighbor] = t_g_count
                f_count[neighbor] = g_count[neighbor] + self.heuristic(neighbor[0], neighbor[1], goal[0], goal[1])
        
        return None # Return Nonetype if no valid path has been found to the End Position
    

    # returns the next move coordinates suggested by A-Star
    def next_move(self, wrld:World, x, y):
        start = (x, y)
        goal = wrld.exitcell

        # "Tables" to record frontier, visited nodes, and heuristic values
        frontier = []
        frontier.append(start)
        explored = set()
        came_from = {start: None}
        g_count = {start: 0}
        f_count = {start: self.heuristic(self.x, self.y, goal[0], goal[1])}

        # A-Star Loop
        while frontier:
            # Use node in frontier with smallest F value
            curr = min(frontier, key=lambda x: f_count[x])

            # Start path tracing if current node is the goal
            if curr == goal:
                self.monsterHeuristic(wrld, 2)
                path = self.trace_path(came_from, curr)
                pathLen = len(path)
                #if(pathLen != 1):
                path.pop() # Pop start node (currently occupied by character)
                nextPos = path.pop()
                return (nextPos[0], nextPos[1], pathLen) # return the next node (suggested next move from path)
            
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
                
                monster_near = self.monsterNear(wrld, neighbor[0], neighbor[1], 5)
                if(monster_near[0] and not monster_near[1] == 0):
                    t_g_count += 10/monster_near[1] # 1000000

                
                # check if neigbor is in frontier already or if better t value has been found
                if neighbor not in frontier:
                    frontier.append(neighbor)
                    self.set_cell_color(neighbor[0], neighbor[1], Back.BLUE)
                elif t_g_count >= g_count[neighbor]:
                    continue

                # f_count.update(self.monsterHeuristic(wrld, 3))
                
                # Add node to the came_from dictionary, G values, and F values for frontier search and path reconstruction
                came_from[neighbor] = curr
                g_count[neighbor] = t_g_count
                f_count[neighbor] = g_count[neighbor] + self.heuristic(neighbor[0], neighbor[1], goal[0], goal[1])
        
        return None # Return Nonetype if no valid path has been found to the End Position
    
    ## Functions Unused in current version, for possible future use:
     
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
