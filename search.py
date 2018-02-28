# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

# Raymond Hruby II
# rhruby2
# Konrad Kurzynowski
# kkurzy4
# UIC | CS411 | HW1
# Windows 10

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

#TODO 
def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    root = problem.getStartState()

    if problem.isGoalState(root): return []  # state is already Goal State

    frontier = util.Stack()  # FIFO
    actionList = []
    frontier.push((root, actionList))  # push starting state
    explored = []  # List? faster data structure?

    while not frontier.isEmpty():  # while frontier is not empty
        print "frontier is not empty"
        currState, currActions = frontier.pop()
        explored.append(currState)
        print "added to explored"

        for states in problem.getSuccessors(currState):
            newActions = currActions + [states[1]]

            if not childIsExploredOrIsFrontier(explored, frontier, states[0]):
                if problem.isGoalState(states[0]):
                    print "***FOUND SOLUTION***"
                    for action in newActions:
                        print action
                    return newActions
                frontier.push((states[0], newActions))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    root = problem.getStartState()

    if problem.isGoalState(root): return []  # state is already Goal State

    frontier = util.Queue()  # FIFO
    actionList = []
    frontier.push((root, actionList))  # push starting state
    explored = []  # List? faster data structure?

    while not frontier.isEmpty():  # while frontier is not empty
        print "frontier is not empty"
        currState, currActions = frontier.pop()
        explored.append(currState)
        print "added to explored"

        for states in problem.getSuccessors(currState):
            newActions = currActions + [states[1]]

            if not childIsExploredOrIsFrontier(explored, frontier, states[0]):
                if problem.isGoalState(states[0]):
                    print "***FOUND SOLUTION***"
                    for action in newActions:
                        print action
                    return newActions
                frontier.push((states[0], newActions))

def uniformCostSearch(problem):
    "Search the node of least total cost first."

    root = problem.getStartState()   # start state --> coordinates
    frontier = util.PriorityQueue()  # PriorityQueue
    actionList = []
    frontier.push((root, actionList), 0)  # push starting state and action list
    explored = []  # List? faster data structure?

    while not frontier.isEmpty():  # while frontier is not empty
        currState, currActions = frontier.pop()  # popping lowest state

        if problem.isGoalState(currState):
            print "***FOUND SOLUTION***"
            for action in currActions:
                print action
            return currActions

        if currState not in explored:
            for states in problem.getSuccessors(currState):
                # states[0] = coordinates (game state) / states[1] = direction / states[2] = pathCost
                newActions = currActions + [states[1]]
                newCost = problem.getCostOfActions(newActions)
                if not childIsExploredOrIsFrontierUCS(explored, frontier, states[0], newActions, newCost):
                    frontier.push((states[0], newActions), newCost)
                else:
                    frontier.update((states[0], newActions), newCost)
        explored.append(currState)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    root = problem.getStartState()   # start state --> coordinates
    frontier = util.PriorityQueue()  # PriorityQueue
    actionList = []
    frontier.push((root, actionList), nullHeuristic(root, problem))  # push starting state and action list
    explored = []  # List? faster data structure?

    while not frontier.isEmpty():  # while frontier is not empty
        currState, currActions = frontier.pop()  # popping lowest state

        if problem.isGoalState(currState):
            print "***FOUND SOLUTION***"
            for action in currActions:
                print action
            return currActions

        if currState not in explored:
            for states in problem.getSuccessors(currState):
                # states[0] = coordinates (game state) / states[1] = direction / states[2] = pathCost
                newActions = currActions + [states[1]]
                newCost = problem.getCostOfActions(newActions) + heuristic(states[0], problem)
                if not childIsExploredOrIsFrontierUCS(explored, frontier, states[0], newActions, newCost):
                    frontier.push((states[0], newActions), newCost)
                else:
                    frontier.update((states[0], newActions), newCost)
        explored.append(currState)

def childIsExploredOrIsFrontier(explored, frontier, child):
    exploredCount = explored.count(child)
    frontierCount = frontier.count(child)

    if exploredCount is 0 and frontierCount is 0: return 0
    return 1

def childIsExploredOrIsFrontierUCS(explored,frontier,child,actions,newCost):
    exploredCount = explored.count(child)
    if not frontier.update((child, actions), newCost) and exploredCount is 0:
        return 0
    return 1

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
