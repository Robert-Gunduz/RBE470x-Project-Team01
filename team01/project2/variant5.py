# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../teamNN')
from testcharacter_prj2 import TestCharacter
# from testcharacter import TestCharacter
wins = 0
loss = 0
# Create the game
for i in range(0,10):

    random.seed() # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')
    g.add_monster(StupidMonster("stupid", # name
                                "S",      # avatar
                                3, 5,     # position
    ))
    g.add_monster(SelfPreservingMonster("aggressive", # name
                                        "A",          # avatar
                                        3, 13,        # position
                                        2             # detection range
    ))

    # TODO Add your character
    g.add_character(TestCharacter("me", # name
                                "C",  # avatar
                                0, 0  # position
    ))

    # Run!
    g.world.characters[g.world.index(0, 0)][0].pick_file("V5.json")
    g.go(1)
    for event in g.world.events:
        print(event.tpe)
        if event.tpe == 4:
            wins = wins + 1
            break
        elif event.tpe == 3 or event.tpe == 2:
            loss = loss + 1
            break
print("Wins: ", wins)
print("Loss: ", loss)
