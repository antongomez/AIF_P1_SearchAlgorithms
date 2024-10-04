from search import Node
import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.patches import Circle, FancyArrow

failure = Node("failure", path_cost=math.inf)  # Indicates an algorithm couldn't find a solution.
cutoff = Node("cutoff", path_cost=math.inf)  # Indicates iterative deepening search was cut off.


def distance_between_orientations(orientation1, orientation2):
    """Calculate the distance between two orientations"""
    return min((orientation1 - orientation2) % 8, (orientation2 - orientation1) % 8)


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


def generate_random_maps(size, quantity):
    ret_matrices = []
    rep = 0
    while rep < quantity:
        ret_matrices.append(np.random.randint(0, 10, (size, size)))
        rep = rep + 1
    return ret_matrices


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
            action = "rotation" if state[2] != prev_state[2] else "move_forward"
            color = "yellow" if action == "move_forward" else "green"
            plot_arrow(state, color)
            if action == "move_forward":
                circle = Circle((state[1], state[0]), 0.15, color=color, fill=False, linewidth=2)
                ax.add_patch(circle)
            prev_state = state

    ax.grid(False)
    plt.show()
