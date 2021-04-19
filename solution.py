#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Snowman Puzzle domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes and problems
from test_problems import PROBLEMS #20 test problems

##my imports
import math
from timeit import default_timer as timer
import sys

#snowball HEURISTICS
def heur_simple(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  return len(state.snowballs)

def heur_zero(state):
  return 0

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible snowball puzzle heuristic: manhattan distance'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between the snowballs and the destination for the Snowman is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    #Thought process-  give distance of the state specified
    #Manhattan Distance = Math.abs(x1-x0) + Math.abs (y1-y0)
    #get goal co-ordinates (x0, y0) and get location of the state coordinates
    # (x1, y1)
    #goal is state.destination -- gives us the coordinates

    #initialize distance
    distance = 0
    for balls in state.snowballs:
        distance += abs(balls[0] - state.destination[0]) + abs(balls[1] - state.destination[1])
    return distance


def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    distance = 0
    height = state.height - 1
    width = state.width - 1
    dic = state.snowballs
    top = (0, height)
    bottom = (0, 0)
    top_right = (width, height)
    bottom_right = (width, 0)
    for ball in state.snowballs:
      if ball != state.destination:
        if ball == top or ball == bottom or ball == top_right or ball == bottom_right:
            return float("inf")
        if(state.destination[0] != 0) and (ball[0] == 0):
            return float("inf")
        elif(state.destination[1] != 0) and (ball[1] == 0):
            return float("inf")
        elif (state.destination[0] != width) and (ball[0] == width):
            return float("inf")
        elif (state.destination[1] != height) and (ball[1] == height):
            return float("inf")


        if((ball[0]+1, ball[1]) in state.obstacles and (ball[0], ball[1]+1) in state.obstacles) or \
                ((ball[0]-1, ball[1]) in state.obstacles and (ball[0], ball[1]-1) in state.obstacles) or \
                ((ball[0]+1, ball[1]) in state.obstacles and (ball[0], ball[1]-1) in state.obstacles) or \
                ((ball[0]-1, ball[1]) in state.obstacles and (ball[0], ball[1]+1) in state.obstacles):
            return float("inf")

        if (ball[0] == 0) or (ball[0] == width):
            if (ball[0], ball[1]+1) in state.obstacles or (ball[0], ball[1]-1) in state.obstacles:
                return float('inf')
        if (ball[1] == 0) or (ball[1] == height):
            if (ball[0]+1, ball[1]) in state.obstacles or (ball[0]-1, ball[1]) in state.obstacles:
                return float("inf")

      if dic[ball] >= 3:
        if dic[ball] == 3 and ball == state.destination or dic[ball] == 6 and ball == state.destination:
            distance += 0
        elif (dic[ball] == 4 or dic[ball] == 5) and ball != state.destination:
            distance += ((ball[0] - state.destination[0]) ** 2 + (ball[1] - state.destination[1]) ** 2)
            distance += ((ball[0] - state.robot[0]) ** 2) + (ball[1] - state.robot[1]) ** 2
        elif dic[ball] == 6 and ball != state.destination or dic[ball] == 3 and ball != state.destination:
            distance += 3*((ball[0] - state.destination[0]) ** 2 + (ball[1] - state.destination[1]) ** 2)
            distance += 3*((ball[0] - state.robot[0]) ** 2 + (ball[1] - state.robot[1]) ** 2)
      else:
        distance += math.sqrt((ball[0] - state.destination[0]) ** 2 + (ball[1] - state.destination[1]) ** 2)
        distance += math.sqrt((ball[0] - state.robot[0]) ** 2 + (ball[1] - state.robot[1]) ** 2)
    return distance




def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    @param sNode sN: A search node (containing a SnowballState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.

    value = 0
    value += sN.gval + (weight * sN.hval)
    return value


def anytime_gbfs(initial_state, heur_fn, timebound = 5):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    t = os.times()[0]
    time_cost = timebound + t

    def my_fval_fn(sN):
        return fval_function(sN, weight)

    def my_heur_fn(state):
        return heur_alternate(state)

    se = SearchEngine('best_first')
    se.init_search(initial_state, snowman_goal_state, my_heur_fn, my_fval_fn)

    goal = se.search(timebound)
    cost_bound = (float("inf"), float("inf"), float("inf"))

    found = False
    short_goal = False
    while (t < time_cost):
        start = timer()
        if goal:
            found = True
            found = goal
            if(found.gval <= cost_bound[0]):
                cost_bound = (goal.gval, goal.gval, goal.gval*2)
                short_goal = goal
            goal = se.search(timebound)
        if (goal == False):
            return found
        stop = timer()
        t = t - (start - stop)
    return short_goal


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 5):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''

    t = os.times()[0]
    time = timebound + t

    if(initial_state.height > initial_state.width):
        weight = initial_state.height-1
    elif(initial_state.height < initial_state.width):
        weight = initial_state.width-1
    elif(initial_state.height == initial_state.width):
        weight = (initial_state.height * initial_state.width) - initial_state.width

    def my_fval_fn(sN):
        return fval_function(sN, weight)
    search = SearchEngine(strategy='custom', cc_level='full')
    search.init_search(initial_state, snowman_goal_state, heur_fn, my_fval_fn)

    found = False
    goal = search.search(timebound)
    costbound = (float("inf"), float("inf"), float("inf"))

    best_path = False
    while (t < time):
        start = timer()
        if goal:
            found = True
            found = goal
            if(found.gval < costbound[0]):
                costbound = (goal.gval, goal.gval, goal.gval*2)
                best_path = goal
            goal = search.search(timebound)
        if (goal == False):
            short_goal = found
        stop = timer()
        t -= start-stop
    return best_path