"""Run a search algorithm give a map"""

import argparse
from functions import (
    MinningProblem,
    read_input_file,
    breadth_first_graph_search,
    depth_first_graph_search,
    astar_search,
    run_search_algorithm,
)


def main():
    parser = argparse.ArgumentParser(description="Run a search algorithm on a map")

    # Definir los argumentos que quieres aceptar
    parser.add_argument("--input", type=str, required=True, help="Input file path with the map")
    parser.add_argument("--output", type=str, required=False, help="Output file path to save the execution trace")
    parser.add_argument(
        "--algorithm", type=str, required=True, help="Search algorithm to use (bfs, dfs, astar1, astar2)"
    )
    args = parser.parse_args()

    input_file = args.input
    output_file = args.output
    algorithm_name = args.algorithm

    n_rows, n_cols, map_data, initial, goal = read_input_file(input_file)
    mp = MinningProblem(n_rows, n_cols, map_data, initial, goal)

    if algorithm_name == "bfs":
        algorithm = breadth_first_graph_search
    elif algorithm_name == "dfs":
        algorithm = depth_first_graph_search
    elif algorithm_name == "astar1":
        algorithm = lambda mp: astar_search(mp, mp.h1)
    elif algorithm_name == "astar2":
        algorithm = lambda mp: astar_search(mp, mp.h2)
    else:
        raise ValueError("Invalid algorithm. Choose one of: bfs, dfs, astar1, astar2")

    if output_file:
        run_search_algorithm(mp, algorithm, verbose=True, output_file=output_file)
    else:
        run_search_algorithm(mp, algorithm, verbose=True)


if __name__ == "__main__":
    main()
