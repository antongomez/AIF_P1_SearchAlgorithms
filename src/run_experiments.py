"""This scripts runs the 4 algorithms (bfs, dfs, astar1, astar2) with 10 random maps of sizes 3, 5, 7, 9."""

import argparse
from functions import (
    run_experiments,
    print_tables,
)


def main():
    parser = argparse.ArgumentParser(description="Run a search algorithm on a map")

    parser.add_argument("--seed", type=str, required=False, help="Seed for the random number generator")
    parser.add_argument("--table-style", type=str, required=False, default="plain", help="Style for the table output")

    args = parser.parse_args()

    seed = int(args.seed) if args.seed else None
    style = args.table_style

    results = run_experiments(map_sizes=[3, 5, 7, 9], seed=seed)
    print_tables(results, map_sizes=[3, 5, 7, 9], style=style)


if __name__ == "__main__":
    main()
