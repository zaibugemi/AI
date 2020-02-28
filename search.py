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
	return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
	"""
	Search the deepest nodes in the search tree first.

	Your search algorithm needs to return a list of actions that reaches the
	goal. Make sure to implement a graph search algorithm.

	To get started, you might want to try some of these simple commands to
	understand the search problem that is being passed in:
		"""
	# print("Problem ",problem)
	# print("Start:", problem.getStartState())
	# print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
	# print("Start's successors:", problem.getSuccessors(problem.getStartState()))
			
	"Start of Your Code"
	frontier = util.Stack()
	explored_set = set()

	initial_state = problem.getStartState()
	if problem.isGoalState(initial_state):
		return []

	frontier.push((initial_state,[]))
	while True:
		if frontier.isEmpty():
			return []
		
		coord, act_seq = frontier.pop()
		if problem.isGoalState(coord):
			return act_seq

		if coord not in explored_set:
			explored_set.add(coord)
			successors = problem.getSuccessors(coord)
			if successors:
				for a_triple in successors:
					if a_triple[0] not in explored_set:
						path = list()
						if act_seq != []:
							for val in act_seq:
								path.append(val)
						path.append(a_triple[1])
						frontier.push((a_triple[0], path))
	"End of Your Code"

# ________________________________________________________________

class _RecursiveDepthFirstSearch(object):
	'''
		=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
		above. 
		Key Point: Remember in tutorial you were asked to expand the left-most child 
		first for dfs and bfs for consistency. If you expanded the right-most
		first, dfs/bfs would be correct in principle but may not return the same
		path. 

		=> Useful Hint: self.problem.getSuccessors(node) will return children of 
		a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
		is different from 'iterative' traversal, try reversing the sequence.  

	'''
	def __init__(self, problem):
		" Do not change this. " 
		# You'll save the actions that recursive dfs found in self.actions. 
		self.actions = [] 
		# Use self.explored to keep track of explored nodes.  
		self.explored = set()
		self.problem = problem

	def RecursiveDepthFirstSearchHelper(self, node):
		'''
		args: start node 
		outputs: bool => True if path found else Fasle.
		'''
		"Start of Your Code"
		if self.problem.isGoalState(node):
			return True
		else:
			self.explored.add(node)
			successors = self.problem.getSuccessors(node)
			if successors is None:
				return False
			else:
				for a_triple in successors:
					if a_triple[0] not in self.explored:
						self.actions.append(a_triple[1])
						reply = RecursiveDepthFirstSearchHelper(a_triple[0])
						if reply == False:


		else:
			self.explored.add(node)
			successors = self.problem.getSuccessors(node)
			if successors:
				



		"End of Your Code"


def RecursiveDepthFirstSearch(problem):
	" You need not change this function. "
	# All your code should be in member function 'RecursiveDepthFirstSearchHelper' of 
	# class '_RecursiveDepthFirstSearch'."

	node = problem.getStartState() 
	rdfs = _RecursiveDepthFirstSearch(problem)
	path_found = rdfs.RecursiveDepthFirstSearchHelper(node)
	return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def depthLimitedSearch(problem, limit = 210):

	"""
	Search the deepest nodes in the search tree first as long as the
	nodes are not not deeper than 'limit'.

	For medium maze, pacman should find food for limit less than 130. 
	If your solution needs 'limit' more than 130, it's bogus.
	Specifically, for:
	'python pacman.py -l mediumMaze -p SearchAgent -a fn=dls', and limit=130
	pacman should work normally.  

	Your search algorithm needs to return a list of actions that reaches the
	goal. Make sure to implement a graph search algorithm.
	Autograder cannot test this function.  

	Hints: You may need to store additional information in your frontier(queue).

		"""

	"Start of Your Code"
	frontier = util.Stack()
	explored_set = set()
	depth = 0

	initial_state = problem.getStartState()
	if problem.isGoalState(initial_state):
		return []

	frontier.push((initial_state,[],depth))
	while True:
		if frontier.isEmpty():
			return []
		
		coord, act_seq, depth = frontier.pop()
		if problem.isGoalState(coord):
			return act_seq

		if coord not in explored_set:
			explored_set.add(coord)
			successors = problem.getSuccessors(coord)
			if successors:
				if depth <= limit:
					for a_triple in successors:
						if a_triple[0] not in explored_set:
							path = list()
							if act_seq != []:
								for val in act_seq:
									path.append(val)
							path.append(a_triple[1])
							frontier.push((a_triple[0], path, depth + 1))
	"End of Your Code"

# ________________________________________________________________

class _RecursiveDepthLimitedSearch(object):
	'''
		=> Output of 'recursive' dfs should match that of 'iterative' dfs you implemented
		above. 
		Key Point: Remember in tutorial you were asked to expand the left-most child 
		first for dfs and bfs for consistency. If you expanded the right-most
		first, dfs/bfs would be correct in principle but may not return the same
		path. 

		=> Useful Hint: self.problem.getSuccessors(node) will return children of 
		a node in a certain "sequence", say (A->B->C), If your 'recursive' dfs traversal 
		is different from 'iterative' traversal, try reversing the sequence.  

	'''
	def __init__(self, problem):
		" Do not change this. " 
		# You'll save the actions that recursive dfs found in self.actions. 
		self.actions = [] 
		# Use self.explored to keep track of explored nodes.  
		self.explored = set()
		self.problem = problem
		self.current_depth = 0
		self.depth_limit = 204 # For medium maze, You should find solution for depth_limit not more than 204.

	def RecursiveDepthLimitedSearchHelper(self, node):
		'''
		args: start node 
		outputs: bool => True if path found else Fasle.
		'''

		"Start of Your Code"
		pass
		"End of Your Code"


def RecursiveDepthLimitedSearch(problem):
	"You need not change this function. All your code in member function RecursiveDepthLimitedSearchHelper"
	node = problem.getStartState() 
	rdfs = _RecursiveDepthLimitedSearch(problem)
	path_found = rdfs.RecursiveDepthLimitedSearchHelper(node)
	return list(reversed(rdfs.actions)) # Actions your recursive calls return are in opposite order.
# ________________________________________________________________


def breadthFirstSearch(problem):
	"""Search the shallowest nodes in the search tree first."""

	"Start of Your Code"
	frontier = util.Queue()
	explored_set = set()

	initial_state = problem.getStartState()
	if problem.isGoalState(initial_state):
		return []

	frontier.push((initial_state,[]))
	while True:
		if frontier.isEmpty():
			return []
		
		coord, act_seq = frontier.pop()
		if problem.isGoalState(coord):
			return act_seq

		if coord not in explored_set:
			explored_set.add(coord)
			successors = problem.getSuccessors(coord)
			if successors:
				for a_triple in successors:
					if a_triple[0] not in explored_set:
						path = list()
						if act_seq != []:
							for val in act_seq:
								path.append(val)
						path.append(a_triple[1])
						frontier.push((a_triple[0], path))

		

		

	
	"End of Your Code"


def uniformCostSearch(problem):
	"""Search the node of least total cost first.
	   You may need to pay close attention to util.py.
	   Useful Reminder: Note that problem.getSuccessors(node) returns "step_cost". 

	   Key Point: If a node is already present in the queue with higher path cost, 
	   you'll update its cost. (Similar to pseudocode in figure 3.14 of your textbook.). Be careful, 
	   autograder cannot catch this bug. 
	"""

	"Start of Your Code"
	frontier = util.PriorityQueue()
	frontier_copy = list()
	explored_set = set()
	initial_state = problem.getStartState()
	frontier.push((initial_state,[]),0)
	frontier_copy.append(initial_state)

	while True:
		if frontier.isEmpty():
			return []
		
		node, priority = frontier.pop()
		frontier_copy.remove(node[0])

		act_seq = node[1]
		if problem.isGoalState(node[0]):
			return act_seq

		explored_set.add(node[0])
		successors = problem.getSuccessors(node[0])

		if successors:
			for a_triple in successors:
				successor_coord = a_triple[0]
				successor_direction = a_triple[1]
				successor_path_cost = priority + a_triple[2]
				path = list()
				if act_seq != []:
					for direction in act_seq:
						path.append(direction)
				path.append(successor_direction)
				successor_info = (successor_coord,path)

				if successor_coord not in explored_set and successor_coord not in frontier_copy:
					frontier.push(successor_info, successor_path_cost)
					frontier_copy.append(successor_coord)
				elif frontier.item_present_with_higher_priority(successor_info, successor_path_cost) != None:
					index, c = frontier.item_present_with_higher_priority(successor_info, successor_path_cost)
					frontier.Update_priority(successor_info, successor_path_cost, index, c)
		
	"End of Your Code"

def nullHeuristic(state, problem=None):
	"""
	A heuristic function estimates the cost from the current state to the nearest
	goal in the provided SearchProblem.  This heuristic is trivial.
	"""
	return 0

def aStarSearch(problem, heuristic=nullHeuristic):
	'''
	Pay clos attention to util.py- specifically, args you pass to member functions. 

	Key Point: If a node is already present in the queue with higher path cost, 
	you'll update its cost (Similar to pseudocode in figure 3.14 of your textbook.). Be careful, 
	autograder cannot catch this bug.

	'''
	"Start of Your Code"
	frontier = util.PriorityQueue()
	explored_set = set()
	frontier_copy = list()
	initial_state = problem.getStartState()
	frontier.push((initial_state,[],0),heuristic(initial_state,problem))
	frontier_copy.append(initial_state)

	while True:
		if frontier.isEmpty():
			return []
		
		node, fn = frontier.pop()
		frontier_copy.remove(node[0])

		act_seq = node[1]
		gn = node[2]
		if problem.isGoalState(node[0]):
			return act_seq

		explored_set.add(node[0])
		successors = problem.getSuccessors(node[0])

		if successors:
			for a_triple in successors:
				successor_coord = a_triple[0]
				successor_direction = a_triple[1]
				successor_path_cost = gn + a_triple[2] + heuristic(successor_coord,problem)
				path = list()
				if act_seq != []:
					for direction in act_seq:
						path.append(direction)
				path.append(successor_direction)
				successor_info = (successor_coord,path,a_triple[2] + gn)

				if successor_coord not in explored_set and successor_coord not in frontier_copy:
					frontier.push(successor_info,successor_path_cost)
					frontier_copy.append(successor_coord)
				elif frontier.item_present_with_higher_priority(successor_info, successor_path_cost) != None:
					index, c = frontier.item_present_with_higher_priority(successor_info, successor_path_cost)
					frontier.Update_priority(successor_info, successor_path_cost, index, c)
	"End of Your Code"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
rdfs = RecursiveDepthFirstSearch
dls = depthLimitedSearch
rdls = RecursiveDepthLimitedSearch
astar = aStarSearch
ucs = uniformCostSearch
