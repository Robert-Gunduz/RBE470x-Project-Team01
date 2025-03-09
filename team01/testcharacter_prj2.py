# This is necessary to find the main code
from asyncio import PriorityQueue
import sys
import json

from sensed_world import SensedWorld
from world import World
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity, MonsterEntity
from events import *
from colorama import Fore, Back

class TestCharacter(CharacterEntity):

    # Toggle Debug Statements:
    DEBUG = False
    FILETRAIN = True
    filename = 'test.json'
    with open(filename, 'r') as file:
            Qweights = json.load(file)
    # Weights for Q-Learning and other values (exit, monster, explosion)
    Qweights = [4, -1]
    if(FILETRAIN == True):
        with open(filename, 'r') as file:
            Qweights = json.load(file)
    learningRate = 0.1
    discountFactor = 0.8

    # Runs when it is this Character's turn 
    def do(self, wrld):
        # put AI-behavior code HERE:

        # Important variables
        MySquare = (self.x, self.y)
        ExitSquare = self.exit_location(wrld)
        MonsterLocations = self.monster_locations(wrld)
        WallLocations = self.wall_locations(wrld)
        PathToExit = self.A_star(wrld, MySquare[0], MySquare[1], ExitSquare[0], ExitSquare[1])
        if len(PathToExit) == 0:
            isPathOpen = False
        else:
            isPathOpen = True
        if(self.DEBUG):
            print("My Position: ", MySquare)
            print("Goal Position: ", ExitSquare)
            print("Monster Positions: ", MonsterLocations)
            print("Wall Positions: ", WallLocations)
            print("Open path to Exit?: ", isPathOpen)
            print("")
            print("Q_weights: ", self.Qweights)
            print("Bomb Timer: ", self.bomb_timer(wrld))

        # state machine 
        state = self.state_machine(wrld)
        match state:
            case 0:
                NextMove = self.move_Q(wrld, self.x, self.y)
                self.Q_Update(wrld, NextMove[0], NextMove[1])
                self.move(NextMove[0] - self.x, NextMove[1] - self.y)
                pass
            case 1:
                self.place_bomb()
                NextMove = self.move_Q(wrld, self.x, self.y)
                self.Q_Update(wrld, NextMove[0], NextMove[1])
                self.move(NextMove[0] - self.x, NextMove[1] - self.y)
                pass
            case 2:
                PathToExit = self.A_star(wrld, self.x, self.y, self.exit_location(wrld)[0], self.exit_location(wrld)[1])
                if PathToExit:
                    NextMove = PathToExit[-1]  # Move to next step in A* path
                    self.move(NextMove[0] - self.x, NextMove[1] - self.y)
                else:
                    print("WARNING: no path found with A*!")
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
    
    # Helper to detect if cell is in bomb range:
    def danger_zone(self, wrld, x, y):
        bomb = (-1, -1)
        value = False
        bombTimeThreshold = 2
        for i in range(0, wrld.width()):
            for j in range(0, wrld.height()):
                if wrld.bomb_at(i, j):
                    bomb = (i, j)
        if(x == bomb[0]):
            if(abs(y - bomb[1]) < (wrld.expl_range + 2) and self.bomb_timer(wrld) <= bombTimeThreshold):
                value = True
        if(y == bomb[1]):
            if(abs(x - bomb[0]) < (wrld.expl_range + 2) and self.bomb_timer(wrld) <= bombTimeThreshold):
                value = True
        return value

    # Function to check for valid neighboring Cells, returns list of coordinates (modified from 'look_for_empty_cell')
    def neighbors(self, wrld, x, y):
        sensed = SensedWorld.from_world(wrld)
        (nxt, nxt_events) = sensed.next()
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
                           (wrld.empty_at(x + dx, y + dy) and not self.danger_zone(wrld, x + dx, y + dy))):
                            # Yes
                            cells.append((x + dx, y + dy))
        # All done
        return cells
    
    # Function to check for valid neighboring Cells and neighboring cells occupied by walls 
    def all_neighbors(self, wrld, x, y):
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
                           wrld.wall_at(x + dx,y + dy) or
                           not wrld.bomb_at(x + dx,y + dy) or
                           not wrld.explosion_at(x + dx,y + dy) or
                           not wrld.monsters_at(x + dx,y + dy) or
                           not wrld.characters_at(x + dx,y + dy)):
                            # Yes
                            cells.append((x + dx, y + dy, wrld.wall_at(x + dx,y + dy)))
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

    # Get current explosion locations
    def explosion_locations(self, wrld:World):
        explosions = []
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if(wrld.explosion_at(i, j)):
                    explosions.append((i, j))
        return explosions

    # Get current time-steps til bomb explodes, -1 otherwise
    def bomb_timer(self, wrld:World):
        bombs = list(wrld.bombs.values())
        if (len(bombs)):
            bomb:BombEntity = bombs[0]
            return bomb.timer
        return -1

    # Get time of explosion
    def explosion_timer(self, wrld:World):
        return wrld.expl_duration
    
    # Function to reconstruct path (for A-Star)
    def trace_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            self.set_cell_color(current[0], current[1], Fore.RED + Back.GREEN)
            current = came_from[current]
        return path
    
    # Fucntion to pick a file
    def pick_file(self, name):
        self.filename = name
        with open(self.filename, 'r') as file:
            self.Qweights = json.load(file)

    # Function to determine if monster is along the path to the exit
    def monster_along_path(self, wrld):
        state = 0
        ExitSquare = self.exit_location(wrld)
        MonsterLocations = self.monster_locations(wrld)
        if (MonsterLocations):
            closest_monster = min(
            MonsterLocations,
            key=lambda m: len(s3f.A_star(wrld, self.x, self.y, m[0], m[1])) if self.A_star(wrld, self.x, self.y, m[0], m[1]) else float('inf')
            )
            ExitPath = self.A_star(wrld, self.x, self.y, ExitSquare[0], ExitSquare[1])
            MonsterPath = self.A_star(wrld, self.x, self.y, closest_monster[0], closest_monster[1])
            if not ExitPath or not MonsterPath:
                state = 0
            else:
                # create vectors from A* paths
                V_exit = (ExitSquare[0] - self.x, ExitSquare[1] - self.y)
                V_monster = (closest_monster[0] - self.x, closest_monster[1] - self.y)
                # compute magnitudes
                mag_exit = len(ExitPath)
                mag_monster = len(MonsterPath)
                # compute dot product
                dot_product = V_exit[0] * V_monster[0] + V_exit[1] * V_monster[1]
                # compute cos(theta)
                cos_theta = dot_product / (mag_exit * mag_monster) if mag_exit * mag_monster != 0 else -1  # Avoid division by zero
                # condition
                if cos_theta > 0:
                    state = 0  # Q-learning
                else:
                    state = 2  # A* escape
        return state

    ### State Machine ###

    # Function for State change conditions
    def state_machine(self, wrld):
        state = 0 # default state (currently: A-Star)
        if(self.open_hole(wrld, self.x, self.y)):
            state = 1 # drop a bomb state
        elif(self.monster_along_path(wrld) == 2):
            state = 2  # A* state
        return state
    
    # Function to detect when to drop a bomb
    def open_hole(self, wrld, x, y):
        Move_Dict = {}
        destinations = self.all_neighbors(wrld, x, y)
        for move in destinations:
            Move_Dict[move] = self.Q_value(wrld, move[0], move[1])
        bestLoc = max(destinations, key=lambda x: Move_Dict[x])
        return (bestLoc[2])

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
                f_count[neighbor] = g_count[neighbor] + self.chebyshev(neighbor[0], neighbor[1], goal[0], goal[1])

        return []  # Return an empty list if no path to the exit could be found
    
    ### Components for Reward function ###
    def Rewards(self, wrld, x, y):
        reward = 0.01
        sensed = SensedWorld.from_world(wrld)
        (nxt, nxt_events) = sensed.next()
        s_world = SensedWorld.from_world(wrld)
        characters = list(sensed.characters.values())
        characterPoses = []

        exitlocation = self.exit_location(wrld)
        de = self.euclidian(x, y, exitlocation[0], exitlocation[1])
        monsterlocations = self.monster_locations(nxt)
        dms = []
        for monster in monsterlocations:
            dms.append(self.euclidian(x, y, monster[0], monster[1]))
        if not len(dms) == 0:
            dm = min(dms)
        else:
            dm = 999
        # generate list of Monster coordinates from sensed world
        for character in characters:
            temp = character[0]
            characterPoses.append((temp.x, temp.y))
        
        me = characterPoses[0]
        print("D_exit ", de)
        print("D_monster ", dm)
        if(dm == 1):
            reward = -100
            print("Reward type: Killed by Monster!")
        if(de == 0):
            reward = 10
            print("Reward type: Found Exit!")
        print("reward: ", reward)
        print(sensed.events)
        return reward

    ### Components for Q values ###

    # Q Function for Exit
    def Q_exit(self, wrld, x, y):
        exitlocation = self.exit_location(wrld)
        de = self.euclidian(x, y, exitlocation[0], exitlocation[1])
        #print("dist to Exit: ", de)
        return 1/(de + 1)
    
    # Q Function for Monsters
    def Q_monster(self, wrld, x, y):
       monsterlocations = self.monster_locations(wrld)
       dm = 999
       dms = []
       for monster in monsterlocations:
           dms.append(self.euclidian(x, y, monster[0], monster[1]))
       if not len(dms) == 0:
           dm = min(dms)
           #print("dist to Monster: ", dm)
       else:
           return 0
       return 1/(dm + 1)
    
    # Q Function for Explosions
    def Q_explosions(self, wrld, x, y): # Unsure if will work
       explosionlocations = self.explosion_locations(wrld)
       dx = 999
       dxs = []
       for explosion in explosionlocations:
           dxs.append(self.euclidian(x, y, explosion[0], explosion[1]))
       if not len(dxs) == 0:
           dx = min(dxs)
           return 1/(dx + 1)
       else:
           return 0
    
    # Q Function for Bomb (along cardinal directions)
    def Q_bomb(self, wrld, x, y):
        bomb = (-1, -1)
        db = 999
        for i in range(0, wrld.width()):
            for j in range(0, wrld.height()):
                if wrld.bomb_at(i, j):
                    bomb = (i, j)
        dbs = []
        if not bomb == (-1, -1):
            #dbs = [self.euclidian(i, 0, x, 0), self.euclidian(0, j, 0, y)]
            dbs = [abs(i - x), abs(j - y)]
        else:
            return 0
        if not len(dbs) == 0:
            db = min(dbs)
        #print("Q_bomb: ", (x, y, 1/(db + 1)))
        return 1/(db + 1)

    
    # Approximate Q function for given coordinate
    def Q_value(self, wrld, x, y):
        We = self.Qweights[0]
        Wm = self.Qweights[1]
        value = (We * self.Q_exit(wrld, x, y)) + (Wm * self.Q_monster(wrld, x, y)) #+ (Wx * self.Q_explosions(wrld, x, y)) + (Wb * self.Q_bomb(wrld, x, y))
        # print("Q_value: ", (x, y, value))
        return value
    
    # Function to move to best Q-value:
    def move_Q(self, wrld, x, y):
        #print("Q_move:")
        Move_Dict = {}
        destinations = self.neighbors(wrld, x, y)
        if(len(destinations) == 0):
            return (x, y)
        for move in destinations:
            Move_Dict[move] = self.Q_value(wrld, move[0], move[1])
        return max(destinations, key=lambda x: Move_Dict[x])
    
    def Q_Update(self, wrld, x, y):

        reward = self.Rewards(wrld, x, y)
        #print("Reward: ", reward)
        #print("location: ", (self.x, self.y))
        (futureWorld, _) = SensedWorld.from_world(wrld).next()
        neighbors = self.neighbors(futureWorld, x, y)
        Qmax = 0
        for neighbor in neighbors:
            temp = self.Q_value(futureWorld, neighbor[0], neighbor[1])
            if(temp > Qmax):
                Qmax = temp
        delta = (reward + self.discountFactor * Qmax) - self.Q_value(wrld, x, y)
        
        self.Qweights[0] = self.Qweights[0] + self.learningRate * delta * self.Q_exit(wrld, x, y)
        self.Qweights[1] = self.Qweights[1] + self.learningRate * delta * self.Q_monster(wrld, x, y)
        
        if(self.FILETRAIN == True):
            with open(self.filename, 'w') as file:
                json.dump(self.Qweights, file)

