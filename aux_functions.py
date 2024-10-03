from search import Node
import math

failure = Node("failure", path_cost=math.inf)  # Indicates an algorithm couldn't find a solution.
cutoff = Node("cutoff", path_cost=math.inf)  # Indicates iterative deepening search was cut off.


def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None):
        return []
    return path_states(node.parent) + [node.state]


def run_search_algorithm(problem, algorithm):
    print(
        f"#################################\nRunning {algorithm.__name__} algorithm\n#################################"
    )
    node = algorithm(problem)
    print(f"\nPath cost: {node.path_cost}")
    if node == failure:
        print("Failure")
    elif node == cutoff:
        print("Cutoff")
    else:
        print("\nPath actions:")
        print(f"State {0}: {problem.initial}")
        for i, state_action in enumerate(zip(path_states(node)[1:], path_actions(node))):
            print(f"Action {i}: {state_action[1]}\nState {i+1}: {state_action[0]}")
