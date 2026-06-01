# Movhex - Route Planner 

**Course**: Algorithms and Data Structures @ Politecnico di Milano

## Overview
This project is my final assignment for the Algorithms and Data Structures course. 
It is a C program that computes optimal routes on a dynamic hexagonal grid for a fictional transportation company.

## Features
The program processes commands from standard input to update the map and calculate paths:
* `init`: Initializes the hexagonal grid.
* `change_cost`: Updates the traversal costs of a specific circular area.
* `toggle_air_route`: Adds or removes unidirectional connections between specific hexagons.
* `travel_cost`: Calculates the minimum travel cost between two points.

## Implementation Details
To meet the strict time and memory constraints of the assignment, the following data structures and algorithms were used:
* **Pathfinding:** Implemented Dijkstra's algorithm with a custom Min-Heap priority queue.
* **Graph Structure:** Adjacency list graph with an array-based memory pool (`EdgePool`) to manage the edges efficiently.
* **Optimization:** Added a basic hash-map caching system to save and reuse the results of frequent `travel_cost` queries.
* **Coordinates:** Converted 2D grid coordinates into 3D cubic coordinates (x, y, z) to simplify distance calculations between hexagons.
