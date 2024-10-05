# P1_SearchAlgorithms

This repository is part of the first assignment for the Artificial Intelligence Fundamentals course. The objective of this assignment is to compare the performance of various search algorithms on a grid map and develop two heuristics for the A\* algorithm.

## File Structure

The repository is organized as follows:

- `data/`: Contains the input file, `exampleMap.txt`, which defines the grid map used for testing.
- `src/`: Contains the project's source code. The Python scripts in this folder include:
  - `run_algorithm_with_map.py`: Executes a specified search algorithm on a given map.
  - `run_experiments.py`: Runs four different search algorithms on 40 randomly generated maps of 4 different sizes.
  - `functions.py`: Provides functions for reading maps, defines the problem class, and includes utilities for running experiments.
  - `p1_search.ipynb`: A Jupyter notebook demonstrating examples of how to use the functions from the above script files.
  - `search.py`: Implements search algorithms based on the AIMA Python repository.
  - `utils.py`: Auxiliary functions used in `search.py`.

## Run Algorithm with Map

To run a search algorithm on a map, execute the following command in the terminal:

```bash
python src/run_algorithm_with_map.py --input data/exampleMap.txt --algorithm astar1 --output output/exampleMap_astar1.txt
```

The `--input` argument specifies the input file containing the map, the `--algorithm` argument specifies the search algorithm to use, and the `--output` argument specifies the output file to save the results. The available algorithms are `bfs`, `dfs`, `astar1`, and `astar2`. If another algorithm is specified, the program will raise an error.

The flag `--output` is optional. If not provided, the results will be printed to the terminal.

## Reproducing the Results

It is possible to reproduce the experiments with the 4 algorithms (BFS, DFS, UCS, and A\*) by running the `run_experiments.py` script.

To run the experiments, execute the following command in the terminal:

```bash
python src/run_experiments.py --seed 42
```

The `--seed` argument is optional and sets the seed for the random number generator. The default value is `None`.
