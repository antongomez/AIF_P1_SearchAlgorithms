"""This module contains the functions to run the experiments and the search algorithms."""

from search import Problem, Node, breadth_first_graph_search, depth_first_graph_search, astar_search

import math

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.patches import Circle, FancyArrow

failure = Node("failure", path_cost=math.inf)  # Indicates an algorithm couldn't find a solution.
cutoff = Node("cutoff", path_cost=math.inf)  # Indicates iterative deepening search was cut off.


def distance_between_orientations(orientation1, orientation2):
    """Calculate the distance between two orientations"""
    return min((orientation1 - orientation2) % 8, (orientation2 - orientation1) % 8)


class MinningProblem(Problem):
    """A map with rows and columns and a robot that can move in 8 directions. There is a initial position and a goal position."""

    def __init__(self, rows, columns, map, initial, goal):
        self.map = map
        self.rows, self.columns = rows, columns
        self.initial, self.goal = initial, goal

    def is_valid_state(self, state):
        """Check that the given state is a possible state given a board with rows and columns."""
        return all(x >= 0 for x in state) and state[0] < self.rows and state[1] < self.columns and state[2] < 8

    def actions(self, state):
        """Return the possible movements of the robot given a state."""
        # Get the orientation of the robot
        orientation = state[2]
        # We always can rotate the robot 45 degrees to the left and to the right
        movements = ["rotate_left", "rotate_right"]
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
        """Move the robot to the next state."""
        if action == "rotate_left":
            return (state[0], state[1], (state[2] - 1) % 8)
        elif action == "rotate_right":
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
        """Check if the current state is the goal state."""
        return state[:2] == self.goal[:2]

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1."""
        # We have to distinguish when the state2 is reached by
        # a rotation of the robot or by a movement of the robot
        if action == "rotate_left" or action == "rotate_right":
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
        """For this heurisit we suppose that all the positions of the board has cost 1."""
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


def run_search_algorithm(problem, algorithm, verbose=False, output_file=None):
    """Run a search algorithm from the search module and print the followed path of states and actions."""

    solution = algorithm(problem)
    if verbose:
        text = f"\n#############################################\nRunning {algorithm.__name__} algorithm\n#############################################\n"
        if solution == failure:
            text += "\nFailure"
        elif solution == cutoff:
            text += "\nCutoff"
        else:
            text += f"\nState 0: {problem.initial} (starting node)\n"

            number_of_last_node = len(solution.solution()) - 1
            for i, state_action in enumerate(zip(solution.solution(), solution.path()[1:])):
                text += f"Action {i+1}: {state_action[0]}\nState {i+1}: {state_action[1].state}" + (
                    f" (final node)\n" if i == number_of_last_node else "\n"
                )
            text += (
                f"\nTotal number of items in explored list: {solution.explored_nodes}\n"
                + f"Total number of items in frontier: {solution.frontier_nodes}\n"
            )

        if output_file:
            with open(output_file, "a") as f:
                f.write(text)
        else:
            print(text)

    return solution


def generate_random_maps(size, quantity, seed=None):
    """Generate <quantity> random maps of size x size."""
    if seed:
        np.random.seed(seed)
    return [np.random.randint(0, 10, (size, size)) for _ in range(quantity)]


def run_experiments(map_sizes=[3, 5, 7, 9], seed=None):
    """Run the experiments for the different search algorithms and map sizes."""
    algorithms = {
        "Breadth-first": breadth_first_graph_search,
        "Depth-first": depth_first_graph_search,
        "A* (h1)": lambda mp: astar_search(mp, mp.h1),
        "A* (h2)": lambda mp: astar_search(mp, mp.h2),
    }

    initial_state = (0, 0, 0)
    results = []

    for size in map_sizes:
        # Define the seed for the random maps to guarantee reproducibility
        seed = seed + 1 if seed else None

        # Generate random maps and define the goal state
        maps = generate_random_maps(size, 10, seed=seed)
        goal = (size - 1, size - 1, 8)

        # Initialize dictionary to store the results of each algorithm
        results_size = {algo: np.zeros((10, 4)) for algo in algorithms}

        for i, map_data in enumerate(maps):
            mp = MinningProblem(size, size, map_data, initial_state, goal)

            # Run the search algorithms
            for algorithm_name, algorithm_fn in algorithms.items():
                solution = algorithm_fn(mp)
                results_size[algorithm_name][i] = (
                    solution.depth,
                    solution.path_cost.item(),
                    solution.explored_nodes,
                    solution.frontier_nodes,
                )

        results.append(results_size)

    return results


def print_tables(results, map_sizes=[3, 5, 7, 9], style="markdown"):
    """Prints the tables of the results of the experiments in markdown format."""
    assert len(results) == len(map_sizes), "The length of results and map_sizes should be the same."
    bfs = "Breadth-first"
    dfs = "Depth-first"
    a_star_h1 = "A* (h1)"
    a_star_h2 = "A* (h2)"

    for map_size, res_size in zip(map_sizes, results):
        print(f"\n#### Comparative table of performance of search methods in the map of dimension {map_size}")
        table = pd.DataFrame(
            {
                bfs: np.mean(res_size[bfs], axis=0),
                dfs: np.mean(res_size[dfs], axis=0),
                a_star_h1: np.mean(res_size[a_star_h1], axis=0),
                a_star_h2: np.mean(res_size[a_star_h2], axis=0),
            },
            index=["d", "g", "#E", "#F"],
        )
        # Transpose the table to make it easier to read
        transposed_table = table.transpose()
        if style == "markdown":
            print(transposed_table.to_markdown())
        elif style == "html":
            print(transposed_table.to_html())
        elif style == "latex":
            print(transposed_table.to_latex())
        else:
            print(transposed_table)


######################################
# I/O functions the maps from a file #
######################################


def read_input_file(filename):
    """Read the input file and return the list of lines."""
    with open(filename, "r") as file:
        n_rows, n_cols = [int(x) for x in file.readline().split(" ")]

        # Read the map
        map_costs = []

        for _ in range(n_rows):
            map_costs.append([int(x) for x in file.readline().split(" ")])

        # Read the initial and final positions
        initial = tuple([int(x) for x in file.readline().split(" ")])
        goal = tuple([int(x) for x in file.readline().split(" ")])

        return n_rows, n_cols, map_costs, initial, goal


###################################################
# Functions to plot the paths, used in the report #
###################################################


def get_states_h2(state, goal):
    """Calculate the states and cost to move the robot from state to goal using the h2 heuristic."""
    # If the robot is already in the goal state, the cost is and there are no steps
    if state[:2] == goal[:2]:
        return 0, [state]

    orientation = state[2]

    movements_y = goal[0] - state[0]
    movements_x = goal[1] - state[1]

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

    # Calculate the total cost
    number_rotations = max(number_rotations_straight, number_rotations_diagonal)
    total_cost = number_rotations + diagonal_movements + straight_movements

    # Calculate the steps
    states = [state]

    def move_with_orientation(state, orientation):
        new_state = list(state)
        if orientation == 0:
            new_state[0] = state[0] - 1
        elif orientation == 1:
            new_state[0] = state[0] - 1
            new_state[1] = state[1] + 1
        elif orientation == 2:
            new_state[1] = state[1] + 1
        elif orientation == 3:
            new_state[0] = state[0] + 1
            new_state[1] = state[1] + 1
        elif orientation == 4:
            new_state[0] = state[0] + 1
        elif orientation == 5:
            new_state[0] = state[0] + 1
            new_state[1] = state[1] - 1
        elif orientation == 6:
            new_state[1] = state[1] - 1
        elif orientation == 7:
            new_state[0] = state[0] - 1
            new_state[1] = state[1] - 1
        return (new_state[0], new_state[1], orientation)

    def append_states(state, orientation, movements, states):
        """Function to append the states to the list of states"""
        for _ in range(movements):
            state = move_with_orientation(state, orientation)
            states.append(state)
        return state

    if diagonal_movements == 0:
        if number_rotations_straight > 0:
            states.append((state[0], state[1], straight_orientation))
        state = append_states(state, straight_orientation, straight_movements, states)
    elif straight_movements == 0:
        if number_rotations_diagonal > 0:
            states.append((state[0], state[1], diagonal_orientation))
        state = append_states(state, diagonal_orientation, diagonal_movements, states)
    else:
        if number_rotations_straight < number_rotations_diagonal:
            if number_rotations_straight > 0:
                states.append((state[0], state[1], straight_orientation))  # rotation
            state = append_states(state, straight_orientation, straight_movements, states)
            states.append((state[0], state[1], diagonal_orientation))  # rotation
            state = append_states(state, diagonal_orientation, diagonal_movements, states)
        else:
            if number_rotations_diagonal > 0:
                states.append((state[0], state[1], diagonal_orientation))  # rotation
            state = append_states(state, diagonal_orientation, diagonal_movements, states)
            states.append((state[0], state[1], straight_orientation))
            state = append_states(state, straight_orientation, straight_movements, states)

    return total_cost, states


def plot_path_in_map(map, normalize=True, initial_state=None, states=None):
    fig, ax = plt.subplots()

    # Define the normalization for the colors
    if normalize:
        norm = Normalize(vmin=np.min(map) - 1, vmax=np.max(map) + 2.5)
    else:
        norm = Normalize(vmin=np.min(map), vmax=np.max(map))

    # Plot the map
    cax = ax.matshow(map, cmap="gray", norm=norm)

    # Add the values of the map
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            ax.text(j, i, str(map[i, j]), va="center", ha="center", color="white")

    ax.set_xticks(np.arange(map.shape[1]))
    ax.set_yticks(np.arange(map.shape[0]))
    ax.set_xticklabels(np.arange(map.shape[1]))
    ax.set_yticklabels(np.arange(map.shape[0]))

    def calculate_start_end_arrow(state):
        """Auxiliary function to calculate the end of the arrow given a state."""
        orientation = state[2]
        if orientation == 0:
            angle = np.pi / 2
        elif orientation == 1:
            angle = np.pi / 4
        elif orientation == 2:
            angle = 0
        elif orientation == 3:
            angle = -np.pi / 4
        elif orientation == 4:
            angle = -np.pi / 2
        elif orientation == 5:
            angle = -3 * np.pi / 4
        elif orientation == 6:
            angle = np.pi
        elif orientation == 7:
            angle = 3 * np.pi / 4

        radius = 0.15
        start_x = state[1] + radius * np.cos(angle)
        start_y = state[0] - radius * np.sin(angle)

        radius = 0.25
        end_x = radius * np.cos(angle)
        end_y = -radius * np.sin(angle)

        return start_x, start_y, end_x, end_y

    def plot_circle(state, color):
        circle = Circle((state[1], state[0]), 0.15, color=color, fill=False, linewidth=2)
        ax.add_patch(circle)

    def plot_arrow(state, color):
        start_x, start_y, end_x, end_y = calculate_start_end_arrow(state)
        arrow = FancyArrow(
            start_x,
            start_y,
            end_x,
            end_y,
            width=0.01,
            color=color,
        )
        ax.add_patch(arrow)

    # Circle the initial position
    if initial_state:
        plot_circle(initial_state, "red")

    # Add the actions
    if states:
        # Plot the initial orientation with a red arrow
        prev_state = states[0]
        plot_arrow(prev_state, "red")
        # Plot the rest of the actions
        for state in states[1:]:
            action = "rotate" if state[2] != prev_state[2] else "move_forward"
            color = "yellow" if action == "move_forward" else "green"
            plot_arrow(state, color)
            if action == "move_forward":
                circle = Circle((state[1], state[0]), 0.15, color=color, fill=False, linewidth=2)
                ax.add_patch(circle)
            prev_state = state

    ax.grid(False)
    plt.show()
