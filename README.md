# APFs for MAPF

This project provides a framework for implementing, testing, and evaluating various algorithms for Multi-Agent Path Finding (MAPF) and Lifelong MAPF (LMAPF).

## Overview

The primary goal of this project is to serve as a research and educational tool for studying and comparing different MAPF and LMAPF algorithms. It includes implementations of several popular algorithms, a set of diverse maps for testing, and scripts for running experiments and visualizing the results.

## Features

- **Algorithm Implementation:** A collection of classic and modern MAPF and LMAPF algorithms.
- **Experimentation Framework:** Scripts to run batch experiments with different algorithms, maps, and agent configurations.
- **Result Analysis:** Tools to analyze and visualize the performance of the algorithms.
- **Diverse Map Set:** A variety of maps, including empty, random, and room-based environments.

## Implemented Algorithms

The `algs` directory contains the implementations of the following algorithms:

- **MAPF Algorithms:**
    - Priority-Based Search (PBS)
    - Conflict-Based Search (CBS)
    - LaCAM
    - LaCAM*
    - LNS2
    - PIBT
    - PrP
- **Lifelong MAPF Algorithms:**
    - Lifelong LaCAM
    - Lifelong LaCAM*
    - Lifelong LNS2
    - Lifelong PIBT
    - Lifelong PrP

## Directory Structure

- `algs/`: Contains the core implementations of the MAPF and LMAPF algorithms.
- `final_logs/`: Stores the results of the experiments in JSON format.
- `maps/`: Contains the map files used for the experiments.
- `experiments_MAPF.py`: Script for running MAPF experiments.
- `experiments_LMAPF.py`: Script for running Lifelong MAPF experiments.
- `show_results.py`: Script for visualizing and analyzing the experiment results.

## Usage

### Running Experiments

To run the MAPF experiments, execute the following command:

```bash
python experiments_MAPF.py
```

To run the Lifelong MAPF experiments, use:

```bash
python experiments_LMAPF.py
```

### Visualizing Results

To visualize the results of the experiments, run:

```bash
python show_results.py
```

## Dependencies

This project requires Python 3 and the following libraries:

- `numpy`
- `matplotlib`
- `imageio`

Install the dependencies using pip:

```bash
pip install numpy matplotlib imageio
```

## Contributing

Contributions to this project are welcome. Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.