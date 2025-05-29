Berikut adalah versi *README.md* yang sudah dirapikan dan ditingkatkan, siap kamu salin langsung ke GitHub:

````markdown
# Pathfinding Visualization Project

An interactive Python-based visualization of classic pathfinding algorithms, built with **Pygame**.

---

## 👥 Collaborators

**Class:** DAA (G)

- Ahmad Zaky Ash Shidqi - 5025231229  
- Muhammad Daffa Rizky Sutrisno - 5025231207  
- Rafif Thariq Dhiyaulhaqi - 5025231  

---

## 🧠 Algorithms Visualized

### Greedy Best-First Search
![Greedy BFS](https://github.com/user-attachments/assets/3f3a7462-8b4f-4d6a-a929-84e589bf8136)

### A* Search
![A* Search](https://github.com/user-attachments/assets/9dcf1bc4-3e63-4971-88ec-dcc399a79d45)

### Dijkstra's Algorithm
![Dijkstra](https://github.com/user-attachments/assets/eea5bf28-4775-411d-a984-2a21bc7b3565)

### Depth-First Search (DFS)
![DFS](https://github.com/user-attachments/assets/5453be61-f41a-4bc9-b984-41fa77bfe378)

---

## 📌 Overview

This project visualizes the behavior of four popular pathfinding algorithms on a dynamic grid:

- **A\* Search**
- **Dijkstra's Algorithm**
- **Greedy Best-First Search**
- **Depth-First Search (DFS)**

The grid includes barriers and dynamically generated mazes using recursive backtracking or random placement. Users can interact with the grid to place start/end nodes, barriers, and trigger visualizations in real-time.

---

## 🧱 Main Components

### 🟦 Grid System

The `Node` class encapsulates logic for each cell:
- Position, pixel coordinates
- State (start, end, barrier, path, visited)
- Neighbor handling and state updates

Grid is initialized using:
```python
def make_grid(rows, width):
    gap = width // rows
    return [[Node(i, j, gap, rows) for j in range(rows)] for i in range(rows)]
````

---

### ⚙️ Algorithms

Implemented algorithms use appropriate data structures (stack, queue, or priority queue), track visited nodes, and reconstruct paths using a `came_from` dictionary.

#### Example: A\* Search

```python
def astar(draw, grid, start, end):
    ...
```

Heuristics:

* `g_score`: Distance from start
* `f_score = g_score + h`: Estimated total cost (Manhattan heuristic)

---

### 🧩 Maze Generation

Two options:

* **Recursive Backtracking**: Guarantees a solvable maze.
* **Random Walls**: Adds variability and randomness.

Walls are set using `node.make_barrier()` and rendered black.

---

## 🖼️ Visualization & UI

* Built with **Pygame**
* Dynamic color-coded grid:

  * 🔴 Start
  * 🟢 End
  * ⚫ Wall
  * 🔵 Visited
  * 🟣 Path
* Mouse input:

  * Left click: Place nodes/walls
  * Right click: Remove nodes/walls
* Keyboard input:

  * `Space`: Run algorithm
  * `C`: Clear grid
  * `M`: Generate maze

---

## 🎨 Color Key

| Color      | Meaning             |
| ---------- | ------------------- |
| White      | Empty cell          |
| Red        | Start node          |
| Green      | End node            |
| Black      | Barrier / Wall      |
| Light Blue | Explored node       |
| Purple     | Final path          |
| Grey lines | Grid lines/boundary |

---

## ✅ Evaluation

### Strengths

* Modular design, easy to extend
* Real-time feedback for learning
* Interactive and user-friendly interface

### Limitations

* DFS may not find optimal paths
* Rendering performance may degrade on large grids

---

## 📌 Conclusion

This tool helps visualize and compare multiple pathfinding strategies in an educational, interactive, and fun environment. It's ideal for learning algorithm behavior and can be further enhanced with:

* Weighted tile controls
* Step-by-step visualization
* Performance benchmarking
