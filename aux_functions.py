from search import Node, Problem, depth_first_graph_search, breadth_first_graph_search, astar_search
import math
import numpy as np

failure = Node("failure", path_cost=math.inf)  # Indicates an algorithm couldn't find a solution.
cutoff = Node("cutoff", path_cost=math.inf)  # Indicates iterative deepening search was cut off.


def path_actions(node):
    """The sequence of actions to get to this node."""
    if node.parent is None:
        return []
    return path_actions(node.parent) + [node.action]


def path_states(node):
    """The sequence of states to get to this node."""
    if node in (cutoff, failure, None):
        return []
    return path_states(node.parent) + [node.state]


def run_search_algorithm(problem, algorithm, verbose=False):
    """Run a search algorithm from the search module and print the followed path of states and actions."""

    if verbose:
        print(
            f"#################################\nRunning {algorithm.__name__} algorithm\n#################################"
        )
    node = algorithm(problem)
    if verbose:
        print(f"\nPath cost: {node.path_cost}")
        if node == failure:
            print("Failure")
        elif node == cutoff:
            print("Cutoff")
        else:
            print("\nPath actions:")
            print(f"State {0}: {problem.initial} (initial state)")
            for i, state_action in enumerate(zip(path_states(node)[1:], path_actions(node))):
                print(f"Action {i}: {state_action[1]}\nState {i+1}: {state_action[0]}")
    return node


def random_matrix(size, quantity):
    ret_matrices = []
    rep = 0
    while rep < quantity:
        ret_matrices.append(np.random.randint(0, 10, (size, size)))
        rep = rep + 1
    return ret_matrices


def problem_generator():
    matrix_sizes = [3, 5, 7, 9]
    initial = [0, 0, 0]
    results = []
    for size in matrix_sizes:
        matrices = random_matrix(size, 10)
        goal = [size - 1, size - 1]
        for matrix in matrices:
            mp = MinningProblem(size, size, matrix, initial, goal)
            results.append(breadth_first_graph_search(mp))
            results.append(depth_first_graph_search(mp))
            results.append(astar_search(mp))

    return results


### temporal copy of the problem
class MinningProblem(Problem):
    """We have a board with rows and columns and a robot that can move in 8 directions."""

    def __init__(self, rows, columns, map, initial, goal):
        self.map = map
        self.rows, self.columns = rows, columns
        self.initial, self.goal = initial, goal

    def is_valid_state(self, state):
        """
        Check that the given state is a possible
        state given a board with rows and columns
        """
        return all(x > 0 for x in state) and state[0] < self.rows and state[1] < self.columns and state[2] < 8

    def actions(self, state):
        """Return the possible movements of the robot given a state"""
        # Get the orientation of the robot
        orientation = state[2]
        # We always can rotate the robot 45 degrees to the left and to the right
        movements = ["turn_left", "turn_right"]
        # The startegy to move forward the robot is to move
        # it always and then check if new_state is valid
        # Calculate the new position on axis y
        new_state = list(state)
        if orientation >= 7 or orientation <= 1:
            new_state[0] = state[0] - 1
        elif orientation >= 3 and orientation <= 5:
            new_state[0] = state[0] + 1
        # Calculate the new position on axis x
        if orientation >= 1 and orientation <= 3:
            new_state[1] = state[1] + 1
        elif orientation >= 5 and orientation <= 7:
            new_state[1] = state[1] - 1

        if self.is_valid_state(new_state):
            movements.append("move_forward")

        return movements

    def result(self, state, action):
        """Move the robot to the next state"""
        if action == "turn_left":
            return (state[0], state[1], (state[2] - 1) % 8)
        elif action == "turn_right":
            return (state[0], state[1], (state[2] + 1) % 8)
        elif action == "move_forward":
            new_state = list(state)
            if state[2] >= 7 or state[2] <= 1:
                new_state[0] = state[0] - 1
            elif state[2] >= 3 and state[2] <= 5:
                new_state[0] = state[0] + 1
            if state[2] >= 1 and state[2] <= 3:
                new_state[1] = state[1] + 1
            elif state[2] >= 5 and state[2] <= 7:
                new_state[1] = state[1] - 1
            return tuple(new_state)
        return action

    def goal_test(self, state):
        """Check if the current state is the goal state"""
        return state[:2] == self.goal[:2]

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1."""
        # We have to distinguish when the state2 is reached by
        # a rotation of the robot or by a movement of the robot
        if action == "turn_left" or action == "turn_right":
            return c + 1
        else:
            return c + self.map[state2[0]][state2[1]]

    def h1(self, node):
        """
        Distance of Mahalanobis between the current state and the goal state.
        To elaborate this heuristic, we supose that all the positions of the board
        has cost 1 and the robot can rotate without cost. Also, we suppose that
        the robot only has to move forward in one axis to reach the goal state.
        """
        return max(abs(self.goal[0] - node.state[0]), abs(self.goal[1] - node.state[1]))

    def h2(self, node):
        """For this heurisit we suppose that all the positions of the board
        has cost 1.
        """
        if self.goal_test(node.state):
            return 0

        orientation = node.state[2]

        movements_y = self.goal[0] - node.state[0]
        movements_x = self.goal[1] - node.state[1]

        # Calculate the directions to move the robot to the goal state
        # in both axis
        orientation_y = 4 if movements_y > 0 else 0
        orientation_x = 2 if movements_x > 0 else 6

        # Calculate absolute values:
        movements_y = abs(movements_y)
        movements_x = abs(movements_x)
        # Calculate the number of movements in diagonal and straight
        diagonal_movements = min(movements_y, movements_x)
        straight_movements = abs(movements_y - movements_x)

        # We have two options: rotate the robot to move diagonally towards
        # the goal state, or rotate the robot to move along the axis where
        # the robot is furthest from the goal state.
        if movements_y > movements_x:
            straight_orientation = orientation_y
        else:
            straight_orientation = orientation_x

        if orientation_y == 0 and orientation_x == 6:  # special case
            diagonal_orientation = 7
        else:
            diagonal_orientation = (orientation_y + orientation_x) // 2

        number_rotations_straight = (
            distance_between_orientations(orientation, straight_orientation) if straight_movements > 0 else 0
        )
        number_rotations_diagonal = (
            distance_between_orientations(orientation, diagonal_orientation) if diagonal_movements > 0 else 0
        )

        number_rotations = max(number_rotations_straight, number_rotations_diagonal)
        total_cost = number_rotations + diagonal_movements + straight_movements

        return total_cost

    def h(self, node):
        return self.h2(node)


def distance_between_orientations(a, b):
    # Calculate the distance between two orientations
    return min((a - b) % 8, (b - a) % 8)
