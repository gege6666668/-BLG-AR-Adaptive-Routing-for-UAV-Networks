# BLG-AR

# BLG-AR

This repository provides a simplified MATLAB implementation of the BLG-AR method proposed in the paper:

“Adaptive Routing for UAV Networks Based on Blind Node Localization and Game-Theoretic Path Selection (BLG-AR)”.

## Requirements

- MATLAB R2023a or later

## Repository Structure

- localization.m  
  Blind node localization module.

- adaptive_beacon.m  
  Adaptive beacon interval adjustment module.

- game_routing.m  
  Game-theoretic routing strategy module.

- run_simulation.m  
  Main script for reproducing simulation results.

- datasets/  
  Simulation datasets in CSV format corresponding to the numerical results reported in the manuscript, including:
  - packet delivery ratio
  - throughput
  - routing overhead
  - end-to-end delay
  - energy consumption

## Dataset Description

The datasets were generated through MATLAB-based simulations under different network densities and mobility conditions.

The CSV files correspond to the numerical results used to generate the figures and performance evaluations reported in the manuscript.

The datasets are provided in CSV format under the datasets/ directory. Each file corresponds to one performance metric reported in the manuscript.
## How to Run

Run the following script in MATLAB:

run_simulation.m
