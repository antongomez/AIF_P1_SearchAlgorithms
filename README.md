# Artificial Intelligence Fundamentals - Assignment 1

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
python src/run_algorithm_with_map.py --input data/exampleMap.txt --algorithm astar1 --output outputs/exampleMap_astar1.txt
```

The `--input` argument specifies the input file containing the map, the `--algorithm` argument specifies the search algorithm to use, and the `--output` argument specifies the output file to save the results. The available algorithms are `bfs`, `dfs`, `astar1`, and `astar2`. If another algorithm is specified, the program will raise an error.

The flag `--output` is optional and specifies the output file to save the results. If not provided, the results will be printed to the terminal.

For example, the output file for BFS on the `exampleMap.txt` map will look like this:

```txt

#############################################
Running breadth_first_graph_search algorithm
#############################################

State 0: (0, 3, 0) (starting node)
Action 1: rotate_left
State 1: (0, 3, 7)
Action 2: rotate_left
State 2: (0, 3, 6)
Action 3: rotate_left
State 3: (0, 3, 5)
Action 4: move_forward
State 4: (1, 2, 5) (final node)

Total number of items in explored list: 6
Total number of items in frontier: 3

```

## Reproducing the Results

It is possible to reproduce the experiments with the 4 algorithms (BFS, DFS, UCS, and A\*) by running the `run_experiments.py` script.

To run the experiments, execute the following command in the terminal:

```bash
python src/run_experiments.py --seed 42
```

This scripts runs the four search algorithms on 40 randomly generated maps of 4 different sizes. The results with each algorithm and size are averaged and displayed in a table.

The `--seed` argument is optional and sets the seed for the random number generator. The default value is `None`. The script also has another optional argument, `--style`, which allows control of the style of tables printed in the terminal. The default value is `plain`, but it can be set to `markdown`, `latex`, or `html`.

The output should look like this:

```txt
#### Comparative table of performance of search methods in the map of dimension 3
                 d     g    #E    #F
Breadth-first  5.0  11.9  10.0   6.0
Depth-first    8.0  19.8   8.0   8.0
A* (h1)        5.4  11.1  28.5   8.9
A* (h2)        5.4  11.1  19.5  10.1

#### Comparative table of performance of search methods in the map of dimension 5
                  d     g     #E    #F
Breadth-first   7.0  23.4   33.0  28.0
Depth-first    12.0  38.5   12.0  16.0
A* (h1)        11.2  19.9  122.6  24.3
A* (h2)        11.2  19.9   93.6  31.8

#### Comparative table of performance of search methods in the map of dimension 7
                  d     g     #E    #F
Breadth-first   9.0  35.7  106.0  59.0
Depth-first    16.0  61.1   16.0  24.0
A* (h1)        13.7  25.7  229.9  49.2
A* (h2)        13.5  25.7  179.2  56.1

#### Comparative table of performance of search methods in the map of dimension 9
                  d     g     #E    #F
Breadth-first  11.0  34.4  242.0  91.0
Depth-first    20.0  68.7   20.0  32.0
A* (h1)        15.8  27.5  349.1  73.9
A* (h2)        15.8  27.5  269.0  84.6
```
