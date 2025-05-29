# Greedy BfS

![image](https://github.com/user-attachments/assets/3f3a7462-8b4f-4d6a-a929-84e589bf8136)

# A* Search

![image](https://github.com/user-attachments/assets/9dcf1bc4-3e63-4971-88ec-dcc399a79d45)

# Djikstra

![image](https://github.com/user-attachments/assets/eea5bf28-4775-411d-a984-2a21bc7b3565)

# DFS

![image](https://github.com/user-attachments/assets/5453be61-f41a-4bc9-b984-41fa77bfe378)


# Pathfinding Visualization Project

## Overview

This project presents an interactive visualization of classical pathfinding algorithms, including A\* Search, Dijkstra, Greedy Best-First Search (GBFS), and Depth-First Search (DFS), implemented with a dynamic maze generator using recursive backtracking and a simple random wall generator. The application is built using Python with Pygame, providing a graphical interface for observing algorithm behavior in real-time.

## Design

The application was designed with modularity and educational clarity in mind. The main components include:

- **Grid System**: The `SquareGrid` and `WeightedGrid` classes encapsulate grid logic, including wall handling, weight assignment, and neighbor identification with optional diagonal movement.
- **Pathfinding Algorithms**: Four algorithms are implemented, each representing a distinct strategy:

  - \*_A_ Search\*\*: Combines uniform cost and heuristic search.
  - **Dijkstra**: Considers only cost from start (uniform cost search).
  - **Greedy BFS**: Uses heuristic only.
  - **DFS**: Explores paths to arbitrary depth using a LIFO stack.

- **Maze Generation**:

  - **Recursive Backtracking Maze**: Guarantees solvable mazes with a controlled generation approach.
  - **Simple Random Maze**: Offers variety with randomly placed walls and accessible paths.

- **Visualization**: Pygame handles drawing the grid, walls, nodes, paths, and icons with consistent theming and color-coded highlights for different states (e.g., explored, path, walls).

## Implementation

This project is implemented using **Python and Pygame** to visualize four pathfinding algorithms: A\*, Dijkstra’s Algorithm, Greedy Best-First Search, and Depth-First Search (DFS). The visualization is rendered on a 2D grid, allowing users to interactively place start and end points, barriers (walls), and generate mazes.

The project is modularized into the following components:

1. **Grid and Node Class**
2. **Algorithms Implementation**
3. **Maze Generation**
4. **Main Loop and Pygame Visualization**

---

### **1. Grid System and Node Class**

Each cell on the grid is an instance of the `Node` class. This class encapsulates both the visual and logical behavior of each grid square.

#### **Key attributes of `Node`:**

- `row`, `col`: Position in the grid matrix
- `x`, `y`: Pixel coordinates on the Pygame window
- `color`: Current state (e.g., start, end, barrier, visited)
- `neighbors`: A list of accessible neighboring nodes

#### **Key methods of `Node`:**

```python
def make_start(self): ...
def make_end(self): ...
def make_barrier(self): ...
def reset(self): ...
def is_barrier(self): ...
def update_neighbors(self, grid): ...
def draw(self, win): ...
```

- `update_neighbors(grid)`: Updates the node’s `neighbors` list by checking the grid’s adjacent nodes that are not barriers.
- `draw(win)`: Draws a colored square on the screen representing the node’s current state.

---

### **2. Creating the Grid**

The grid is constructed as a 2D list of `Node` objects:

```python
def make_grid(rows, width):
    gap = width // rows
    grid = [[Node(i, j, gap, rows) for j in range(rows)] for i in range(rows)]
    return grid
```

Here, `gap` defines the pixel size of each cell, and `width` is the window width (e.g., 800px). The grid size can be changed dynamically by modifying `rows`.

---

### **3. Pygame-Based Visualization**

#### **Window Setup**

The Pygame window is initialized using:

```python
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Pathfinding Visualizer")
```

#### **Drawing Functions**

The visual rendering is handled with the following:

```python
def draw_grid(win, rows, width): ...
def draw(win, grid, rows, width): ...
```

- `draw_grid(...)`: Draws grid lines between cells.
- `draw(...)`: Clears the screen, draws each node, and then overlays the grid lines.

Each call to `draw()` represents a visual **frame update**. This is important for animating the progression of the algorithm.

---

### **4. User Interaction and Event Handling**

The main loop uses `pygame.event.get()` to handle:

- Mouse clicks to place the start node, end node, or barriers
- Keyboard input to:

  - Run algorithms (e.g., Space key)
  - Clear the board (`c`)
  - Generate a maze (`m`)

Example event handling:

```python
if pygame.mouse.get_pressed()[0]:
    # Left click to place start/end/wall
elif pygame.mouse.get_pressed()[2]:
    # Right click to erase
```

The grid is updated in real-time based on user input, and every change is reflected by calling `draw(win, grid, rows, width)`.

---

### **5. Pathfinding Algorithms**

Each algorithm is implemented in its own function, all following the same pattern:

- Use a queue (priority queue for A\*, Dijkstra; stack for DFS)
- Track visited nodes
- Use a `came_from` dictionary to reconstruct the final path

#### \*_Example: A_ Search\*\*

```python
def astar(draw, grid, start, end): ...
```

Key logic:

- `g_score`: Cost from start to current node
- `f_score = g_score + h`: Estimated cost to end (heuristic: Manhattan distance)
- Updates colors of nodes in real-time for visualization:

  - Green for open nodes
  - Red for visited
  - Purple for final path

The `draw()` function is called inside the algorithm loop to animate the process.

---

### **6. Path Reconstruction and Animation**

Once the path is found, a backtracking function is used to mark the shortest path from end to start:

```python
def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()
```

Each step of the path is visualized with a color change (e.g., purple) and a call to `draw()` for smooth animation.

---

### **7. Maze Generation**

A maze generator creates a random pattern of barriers with at least one solvable path. It uses either recursive division or randomized DFS. This is helpful for testing the algorithms against non-trivial input.

Maze cells are visually represented with black boxes (`make_barrier()`).

---

### **8. Color Codes for Node Status**

| Color      | Meaning          |
| ---------- | ---------------- |
| White      | Empty node       |
| Dark Blue  | Barrier / Wall   |
| RED        | Start node       |
| Green      | End node         |
| Light Blue | Visited / Closed |
| White      | Final path       |
| Grey lines | Grid boundaries  |

These colors are dynamically updated by calling methods like `make_start()`, `make_end()`, etc., followed by `draw()`.

---

### **9. Summary of Pygame’s Role**

Pygame is responsible for:

- Rendering the grid and its updates (`draw`, `draw_grid`)
- Handling user interaction (mouse, keyboard)
- Animating algorithm progress (calling `draw()` inside loops)
- Displaying maze generation and path reconstruction visually

The seamless visual feedback makes it easier for users to understand how each algorithm behaves in real-time.

## Analysis and Evaluation

### Strengths:

- **Comprehensive Algorithm Coverage**: The project enables comparison of four pathfinding strategies.
- **Maze Diversity**: Multiple generation techniques offer varied testing environments.
- **Real-Time Feedback**: Visual representation provides instant insight into algorithm behavior and performance.
- **Modularity**: Each algorithm and component can be independently tested or extended.

### Limitations:

- **Performance**: Pygame may not scale well for very large grids due to rendering and update overhead.
- **DFS Suboptimality**: As expected, DFS often provides suboptimal paths and may take long routes.
- **Weight Visualization**: Tile weights are shown but their effect might be non-intuitive without numeric labels.

### Evaluation Criteria:

- **Correctness**: All algorithms correctly find a path if one exists.
- **Visual Clarity**: The system clearly illustrates exploration and final paths.
- **Flexibility**: User can switch between algorithms and maze types.

## Conclusion

This project successfully implements and visualizes four major pathfinding algorithms in a modular, interactive framework. The use of recursive maze generation adds complexity and realism, while the visual UI facilitates learning and algorithm comparison. Future improvements could include user-selectable weights, step-by-step execution modes, algorithm performance metrics, and expanded maze types (e.g., hexagonal grids).

This project serves both as an educational tool and a foundation for further AI and game development exploration.
