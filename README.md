# Pathfinding Visualization Project

An interactive Python-based visualization of classic pathfinding algorithms, built with Pygame, featuring a Pac-Man theme. This project demonstrates how different algorithms explore a grid to find a path from a start point (Pac-Man) to an end point (Ghost), and also includes maze generation capabilities.

## Collaborators

Class: DAA (G)

- Ahmad Zaky Ash Shidqi - 5025231229
- Muhammad Daffa Rizky Sutrisno - 5025231207
- Rafif Thariq Dhiyaulhaqi - 5025231213

---

## Project Overview

This application provides a visual and interactive way to understand and compare several fundamental pathfinding algorithms. Users can place walls, set start (Pac-Man) and goal (Ghost) positions, and observe how each algorithm navigates the grid. The project also incorporates maze generation algorithms to create different environments for pathfinding.

---

## Features

-   **Interactive Grid**: Click to add/remove walls, set start and goal points.
-   **Multiple Algorithms**: Switch between A\*, Dijkstra, Greedy BFS, and DFS.
-   **Pac-Man Theme**: Visuals are styled after the classic Pac-Man game.
-   **Maze Generation**: Generate mazes using Recursive Backtracking or a simple random algorithm.
-   **Step-by-Step Visualization**: Shows explored nodes and the final path.
-   **Metrics Display**: Shows path length and steps taken by the algorithm.

---

## Algorithms Implemented

The core of the project lies in the implementation and visualization of these pathfinding algorithms:

### 1. A\* Search

A\* (A-star) is an informed search algorithm, meaning it uses a heuristic to guide its search towards the goal. It aims to find the shortest path by balancing the cost to reach the current node (`g(n)`) and the estimated cost from the current node to the goal (`h(n)`).

-   **Purpose**: Finds the shortest path in a weighted graph, often faster than Dijkstra if a good heuristic is used.
-   **Evaluation Function**: `f(n) = g(n) + h(n)`
    -   `g(n)`: Actual cost from the start node to node `n`.
    -   `h(n)`: Heuristic (estimated cost) from node `n` to the goal. This project uses Manhattan distance.

```python
def a_star_search(graph, start, end):
    """
    A* Search: Dijkstra + Heuristic
    f(n) = g(n) + h(n)
    """
    frontier = PriorityQueue()
    frontier.put(vec2int(start), 0)
    path = {}
    cost = {}
    path[vec2int(start)] = None
    cost[vec2int(start)] = 0

    while not frontier.empty():
        current = frontier.get()
        if current == end:
            break
        for next_node in graph.find_neighbors(vec(current)): # Corrected variable name
            next_node = vec2int(next_node)
            next_cost = cost[current] + graph.cost(current, next_node)
            if next_node not in cost or next_cost < cost[next_node]:
                cost[next_node] = next_cost
                priority = next_cost + heuristic(end, vec(next_node))  # g(n) + h(n)
                frontier.put(next_node, priority)
                path[next_node] = vec(current) - vec(next_node)
    return path, cost
```

### 2. Dijkstra's Algorithm

Dijkstra's Algorithm is another algorithm for finding the shortest paths between nodes in a graph. It explores outwards from the starting node, always choosing the unvisited node with the lowest known distance.

-   **Purpose**: Finds the shortest path in a weighted graph. It's guaranteed to find the shortest path if edge weights are non-negative.
-   **Evaluation Function**: Considers only the actual cost from the start `g(n)`.

```python
def dijkstra_search(graph, start, end):
    """
    Dijkstra Search: Uniform Cost Search
    Uses only g(n) - actual cost
    """
    frontier = PriorityQueue()
    frontier.put(vec2int(start), 0)
    path = {}
    cost = {}
    path[vec2int(start)] = None
    cost[vec2int(start)] = 0

    while not frontier.empty():
        current = frontier.get()
        if current == end:
            break
        for next_node in graph.find_neighbors(vec(current)): # Corrected variable name
            next_node = vec2int(next_node)
            next_cost = cost[current] + graph.cost(current, next_node)
            if next_node not in cost or next_cost < cost[next_node]:
                cost[next_node] = next_cost
                priority = next_cost  # Only g(n)
                frontier.put(next_node, priority)
                path[next_node] = vec(current) - vec(next_node)
    return path, cost
```

### 3. Greedy Best-First Search

Greedy Best-First Search is an informed search algorithm that always selects the path that appears best at that moment based on the heuristic. It attempts to expand the node that is closest to the goal.

-   **Purpose**: Quickly finds a path to the goal, but not necessarily the shortest one.
-   **Evaluation Function**: Uses only the heuristic value `h(n)`.

```python
def greedy_bfs_search(graph, start, end):
    """
    Greedy Best-First Search: Uses only heuristic
    f(n) = h(n)
    """
    frontier = PriorityQueue()
    frontier.put(vec2int(start), 0)
    path = {}
    path[vec2int(start)] = None

    while not frontier.empty():
        current = frontier.get()
        if current == end:
            break
        for next_node in graph.find_neighbors(vec(current)): # Corrected variable name
            next_node = vec2int(next_node)
            if next_node not in path:  # Not visited yet
                priority = heuristic(end, vec(next_node))  # Only h(n)
                frontier.put(next_node, priority)
                path[next_node] = vec(current) - vec(next_node)
    
    # For consistency with other functions, return dummy cost
    cost = {node: 0 for node in path}
    return path, cost
```

### 4. Depth-First Search (DFS)

Depth-First Search explores as far as possible along each branch before backtracking. It uses a stack (Last-In, First-Out) to keep track of nodes to visit.

-   **Purpose**: Finds a path if one exists, but it's not guaranteed to be the shortest or most optimal. Good for exploring all possibilities or for maze solving when any path will do.
-   **Mechanism**: Uses a stack to explore. Neighbors are often shuffled to vary the path found.

```python
def dfs_search(graph, start, end):
    """
    Depth-First Search: Uses a stack (LIFO)
    Not optimal but will find a path if one exists.
    """
    stack = [vec2int(start)]
    path = {}
    visited = set()
    path[vec2int(start)] = None
    
    while stack:
        current = stack.pop()  # LIFO - Last In, First Out
        
        if current in visited:
            continue
            
        visited.add(current)
        
        if current == vec2int(end):
            break
            
        # Shuffle neighbors for path variation
        neighbors = list(graph.find_neighbors(vec(current)))
        random.shuffle(neighbors)
        
        for next_node in neighbors: # Corrected variable name
            next_node = vec2int(next_node)
            if next_node not in visited and next_node not in path:
                stack.append(next_node)
                path[next_node] = vec(current) - vec(next_node)
    
    # Return path with dummy cost for consistency
    cost = {node: 0 for node in path}
    return path, cost
```

---

## Maze Generation

The application includes two methods for generating mazes:

### 1. Recursive Backtracking

This algorithm creates a maze by starting from a cell, choosing a random unvisited neighbor, carving a path to it, and repeating the process. If it hits a dead end, it backtracks.

-   **Characteristics**: Produces mazes with long, winding corridors and relatively few dead ends. It's a common and reliable method.

```python
def generate_recursive_backtrack_maze(width, height):
    """
    Recursive Backtracking Algorithm to generate a maze.
    A stable and reliable algorithm for maze generation.
    """
    # Start with all cells as walls
    walls = set()
    # ... (initialization) ...
    
    # Stack for backtracking
    stack = [current_cell] # Assuming 'current_cell' is initialized
    
    while stack:
        current_cell = stack[-1]
        
        # Find unvisited neighbors
        # ...
        
        if unvisited_neighbors:
            # Choose a random neighbor
            next_cell = random.choice(unvisited_neighbors)
            
            # Remove wall between current_cell and next_cell
            # ...
            
            # Mark as visited and add to stack
            # ...
        else:
            # Backtrack
            stack.pop()
    # ... (ensure start/goal access) ...
    return list(walls)
```

### 2. Simple Maze Generator

This is a more straightforward random maze generator. It primarily places border walls and then randomly adds internal walls with a certain probability.

-   **Characteristics**: Creates more open mazes with potentially many small disconnected areas if the probability is too high, or very sparse mazes if too low. It's less structured than recursive backtracking.

```python
def simple_maze_generator(width, height):
    """
    Simple maze generator as an alternative.
    Creates a maze with a random pattern but tries to ensure a path.
    """
    walls = []
    
    # Create border walls (with consideration for start/goal)
    # ...
    
    # Add random internal walls
    for x in range(1, width - 1):
        for y in range(1, height - 1):
            if random.random() < 0.35:  # 35% chance for an internal wall
                walls.append(vec(x, y))
    
    return walls
```

---

## Controls

-   **Mouse Left Click**: Toggle a wall at the cursor's grid position.
-   **Mouse Middle Click**: Set Pac-Man's (start) position.
-   **Mouse Right Click**: Set Ghost's (goal) position.
-   **SPACE**: Cycle through the available pathfinding algorithms (A\*, Dijkstra, Greedy BFS, DFS).
-   **G**: Generate a new maze using the Recursive Backtracking algorithm.
-   **S**: Generate a new maze using the Simple Maze Generator.
-   **R**: Reset the maze to the original predefined layout.
-   **C**: Clear all walls from the grid.
-   **M**: Dump the current wall list (coordinates) to the console.
-   **ESC**: Quit the application.

---

## How to Run

1.  Ensure you have Python and Pygame installed.
    ```bash
    pip install pygame
    ```
2.  Save the code as a Python file (e.g., `pacman_pathfinding.py`).
3.  Run the file from your terminal:
    ```bash
    python pacman_pathfinding.py
    ```
