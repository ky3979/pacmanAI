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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    visited = []  # List of visited nodes
    path = []  # List of actions to a node

    # DFS uses LIFO Stack
    # Takes (n, path), with
    # n: current node,
    # path: list of actions that leads to n
    fringe = util.Stack()

    # Add start state and initial path to fringe
    fringe.push((problem.getStartState(), path))

    while not fringe.isEmpty():
        # Pop out a node and the path to node
        node, path = fringe.pop()

        if problem.isGoalState(node):
            # This node is the goal, return path
            return path

        # Check if visited node to prevent duplicate visits
        if node not in visited:
            # We just visited this new node
            visited.append(node)

            # Get successors of current node
            successors = problem.getSuccessors(node)

            # If valid adjacent nodes from successor have not been visited,
            # then push the node and the path that leads to it into the fringe
            for next_node, action, _ in successors:
                if next_node not in visited:
                    path_to_node = path + [action]
                    fringe.push((next_node, path_to_node))

    return path


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited = []  # List of visited nodes
    path = []  # List of actions to a node

    # BFS uses FIFO Queue
    # Takes (n, path), with
    # n: current node,
    # path: list of actions that leads to n
    fringe = util.Queue()

    # Add start state and initial path to fringe
    fringe.push((problem.getStartState(), path))

    while not fringe.isEmpty():
        # Dequeue a node and the path to node
        node, path = fringe.pop()

        if problem.isGoalState(node):
            # This node is the goal, return path
            return path

        # Check if visited node to prevent duplicate visits
        if node not in visited:
            # We just visited this new node
            visited.append(node)

            # Get successors of current node
            successors = problem.getSuccessors(node)

            # If valid adjacent nodes from successor have not been visited,
            # then enqueue the node and the path that leads to it into the fringe
            for next_node, action, _ in successors:
                if next_node not in visited:
                    path_to_node = path + [action]
                    fringe.push((next_node, path_to_node))

    return path


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited = []  # List of visited nodes
    path = []  # List of actions to a node
    f = 0   # Cumulative cost to a node

    # UCS uses a Priority Queue
    # Takes (n, path, f), with,
    # n: current node,
    # path: list of actions that leads to n
    fringe = util.PriorityQueue()

    # Add start state, initial path, and initial cost to fringe
    fringe.push((problem.getStartState(), path), f)

    while not fringe.isEmpty():
        # Dequeue a node and the path to node
        node, path = fringe.pop()

        if problem.isGoalState(node):
            # This node is the goal, return path
            return path

        # Check if visited node to prevent duplicate visits
        if node not in visited:
            # We just visited this new node
            visited.append(node)

            # Get successors of current node
            successors = problem.getSuccessors(node)

            # If valid adjacent nodes from successor have not been visited,
            # then enqueue the node, the path to node, and cost to node into fringe
            for next_node, action, _ in successors:
                if next_node not in visited:
                    path_to_node = path + [action]
                    cost_to_node = problem.getCostOfActions(path_to_node)
                    fringe.update((next_node, path_to_node), cost_to_node)

    return path


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = []  # List of visited nodes
    path = []  # List of actions to a node
    f = 0  # Sum of node's cost and heuristic

    # A-Star uses a Priority Queue
    # Takes (n, path, f), with,
    # n: current node,
    # path: list of actions that leads to n,
    fringe = util.PriorityQueue()

    # Add start state, initial path, and initial f to fringe
    fringe.push((problem.getStartState(), path), f)

    while not fringe.isEmpty():
        # Dequeue a node and the path to node
        node, path = fringe.pop()

        if problem.isGoalState(node):
            # This node is the goal, return path
            return path

        # Check if visited node to prevent duplicate visits
        if node not in visited:
            # We just visited this new node
            visited.append(node)

            # Get successors of current node
            successors = problem.getSuccessors(node)

            # If valid adjacent nodes from successor have not been visited,
            # then enqueue the node, the path to node, and sum of cost and heuristic
            for next_node, action, _ in successors:
                if next_node not in visited:
                    path_to_node = path + [action]
                    g = problem.getCostOfActions(path_to_node)  # Cumulative cost to node
                    h = heuristic(next_node, problem)           # Heuristic cost to node
                    f = g + h                                   # Sum of cost and heuristic
                    fringe.update((next_node, path_to_node), f)

    return path


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
