# Greedy Bfs

![image](https://github.com/user-attachments/assets/bbe35d99-d0b7-44a1-bb3d-9d0ad11898a7)

# A Search

![image](https://github.com/user-attachments/assets/dc3fba01-3f84-4f94-9c96-e9592e7d268a)

# Djikstra

![image](https://github.com/user-attachments/assets/74d27b52-35c3-417f-a10d-fa37c1d36e1f)

# Pathfinding Visualizer - A\*, Dijkstra, and Greedy BFS

This project is a pathfinding visualizer that demonstrates how three fundamental algorithms—**A\***, **Dijkstra**, and **Greedy Best-First Search**—explore a grid and find paths from a start point to a goal. It uses **Python** and **Pygame** for real-time visualization.

---

## 📦 Features
- Visualization of:
  - A* Search
  - Dijkstra’s Algorithm
  - Greedy Best-First Search
- Pygame-based animated grid
- Real-time path drawing and node exploration
- Fixed grid with custom wall positions

---

## ## 📘 Detailed Explanation of Algorithms

This project implements and compares three pathfinding algorithms: **A\***, **Dijkstra**, and **Greedy Best-First Search**. All algorithms operate on a 2D grid and attempt to find a path from a **start** node to a **goal** node.

---

### 🔹 A* Search (`a_star_search`)

A\* is a best-first search algorithm that uses a **heuristic function** to estimate the cost from a node to the goal, while also considering the cost from the start to that node.

**Key Concepts:**
- It combines **actual cost (`g(n)`)** from the start and **estimated cost (`h(n)`)** to the goal.
- The total cost `f(n) = g(n) + h(n)` guides the search.
- The heuristic used here is **Manhattan distance**.

**Steps:**
1. Start with the initial node and add it to the open list (priority queue).
2. Always select the node with the lowest `f(n)` value.
3. For each neighbor of the current node:
   - Calculate the new `g(n)` and `f(n)`.
   - If this path is better, update the node and add it to the open list.
4. Stop when the goal node is reached.

**Pros:**
- Finds the **shortest path** efficiently.
- Very effective on grids with obstacles.

---

### 🔹 Dijkstra’s Algorithm (`dijkstra_search`)

Dijkstra’s algorithm is a uniform-cost search. It explores all possible paths in order of increasing cost from the start node.

**Key Concepts:**
- No heuristic is used (`h(n) = 0` for all nodes).
- Only the actual cost `g(n)` is considered.

**Steps:**
1. Start from the initial node with cost `0`.
2. Use a priority queue to pick the node with the smallest known distance.
3. For each neighbor, update the cost if a cheaper path is found.
4. Continue until the goal is reached.

**Pros:**
- **Guaranteed shortest path**.
- Works well for weighted graphs without heuristics.

**Cons:**
- Can be **slower than A\*** because it explores more nodes.

---

### 🔹 Greedy Best-First Search (`greedy_bfs_search`)

Greedy BFS prioritizes nodes based only on how close they seem to the goal (heuristic), ignoring the cost from the start.

**Key Concepts:**
- Only `h(n)` is considered (Manhattan distance in this case).
- Ignores the actual cost `g(n)`.

**Steps:**
1. Start with the initial node.
2. Always select the node that appears **closest to the goal**.
3. For each neighbor, add it to the queue based on its `h(n)` value.
4. Repeat until the goal is found.

**Pros:**
- **Very fast** in open spaces.

**Cons:**
- **Not optimal** — may return longer or incorrect paths.
- Can get stuck or take poor detours in complex grids.

---

### 🔁 Summary Table

| Algorithm            | Uses `g(n)` | Uses `h(n)` | Optimal | Fast |
|----------------------|------------|-------------|---------|------|
| A* Search            | ✅         | ✅          | ✅      | ✅   |
| Dijkstra's Algorithm | ✅         | ❌          | ✅      | ❌   |
| Greedy BFS           | ❌         | ✅          | ❌      | ✅   |


### 5. Visualization
- Colors indicate:
- Explored nodes
- Nodes in the frontier
- Final path
- Start and goal are represented with images or colored squares
- Real-time redraw using Pygame per frame



## 📊 Analysis / Evaluation 

### 🔍 Efficiency
- **A\***: Balanced speed and accuracy; best overall performance
- **Dijkstra**: Slower, explores more nodes, but always finds the shortest path
- **Greedy BFS**: Fastest but may miss optimal routes

### 🧠 Visual Behavior
- **A\***: Smart exploration pattern
- **Dijkstra**: Broad and uniform area coverage
- **Greedy BFS**: Direct movement toward goal, potentially inefficient

### 🔧 Code Structure
- Modular design using Python classes
- Easy to extend with more algorithms or UI features

### ⚠️ Limitations
- Grid size and walls are hard-coded
- No interactive wall placement
- No dynamic resizing or performance profiling

---

## 🔁 Algorithm Comparison Summary

| Function           | Purpose                                                  |
|--------------------|----------------------------------------------------------|
| `a_star_search`     | Efficiently finds the best (optimal and fast) path       |
| `dijkstra_search`   | Finds the cheapest (shortest-cost) path without guessing |
| `greedy_bfs_search` | Fast, but not always optimal or accurate                |



---


## 🧾 Conclusion

Each pathfinding algorithm serves a different purpose based on the trade-off between **speed** and **accuracy**:

- **A\*** is generally the most balanced algorithm, combining the benefits of accuracy and speed using a heuristic and cost-based approach.
- **Dijkstra’s** algorithm is ideal when you need guaranteed shortest paths but are not concerned with execution time.
- **Greedy Best-First Search** is best when speed is the top priority, though it may produce suboptimal paths.


