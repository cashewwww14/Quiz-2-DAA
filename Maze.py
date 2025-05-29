# Pathfinding - Part 5 Enhanced - Pacman Theme
# A* Search, Dijkstra, Greedy BFS, dan DFS dengan Maze Generator
import pygame as pg
from os import path
import heapq
import random
vec = pg.math.Vector2

TILESIZE = 48
GRIDWIDTH = 28
GRIDHEIGHT = 15
WIDTH = TILESIZE * GRIDWIDTH
HEIGHT = TILESIZE * GRIDHEIGHT
FPS = 30

# Pacman Theme Colors
BLACK = (0, 0, 0)                    # Background
BLUE = (0, 0, 255)                   # Maze walls
YELLOW = (255, 255, 0)               # Pacman
WHITE = (255, 255, 255)              # Dots/text
RED = (255, 0, 0)                    # Ghost
ORANGE = (255, 165, 0)               # Ghost
PINK = (255, 192, 203)               # Ghost
CYAN = (0, 255, 255)                 # Ghost
DARK_BLUE = (0, 0, 139)              # Path explored
LIME = (0, 255, 0)                   # Start icon
BACKGROUND = BLACK                    # Pacman background
EXPLORED = (25, 25, 112)             # Dark blue for explored
WALL_COLOR = BLUE                    # Blue maze walls
GRID_COLOR = (40, 40, 40)            # Dark gray grid lines

pg.init()
screen = pg.display.set_mode((WIDTH, HEIGHT))
clock = pg.time.Clock()

# Use pygame default fonts
font_name = pg.font.get_default_font()
def draw_text(text, size, color, x, y, align="topleft"):
    font = pg.font.Font(font_name, size)
    text_surface = font.render(text, True, color)
    text_rect = text_surface.get_rect(**{align: (x, y)})
    screen.blit(text_surface, text_rect)

class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.connections = [vec(1, 0), vec(-1, 0), vec(0, 1), vec(0, -1)]
        # untuk diagonal atau tidak
        self.connections += [vec(1, 1), vec(-1, 1), vec(1, -1), vec(-1, -1)]

    def in_bounds(self, node):
        return 0 <= node.x < self.width and 0 <= node.y < self.height

    def passable(self, node):
        return node not in self.walls

    def find_neighbors(self, node):
        neighbors = [node + connection for connection in self.connections]
        neighbors = filter(self.in_bounds, neighbors)
        neighbors = filter(self.passable, neighbors)
        return neighbors

    def draw(self):
        for wall in self.walls:
            rect = pg.Rect(wall * TILESIZE, (TILESIZE, TILESIZE))
            pg.draw.rect(screen, WALL_COLOR, rect)
            # Add border to walls for more Pacman-like appearance
            pg.draw.rect(screen, WHITE, rect, 2)

class WeightedGrid(SquareGrid):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.weights = {}

    def cost(self, from_node, to_node):
        if (vec(to_node) - vec(from_node)).length_squared() == 1:
            return self.weights.get(to_node, 0) + 10
        else:
            return self.weights.get(to_node, 0) + 14

    def draw(self):
        for wall in self.walls:
            rect = pg.Rect(wall * TILESIZE, (TILESIZE, TILESIZE))
            pg.draw.rect(screen, WALL_COLOR, rect)
            # Add border to walls for more Pacman-like appearance
            pg.draw.rect(screen, WHITE, rect, 2)
        for tile in self.weights:
            x, y = tile
            rect = pg.Rect(x * TILESIZE + 3, y * TILESIZE + 3, TILESIZE - 3, TILESIZE - 3)
            pg.draw.rect(screen, ORANGE, rect)

class PriorityQueue:
    def __init__(self):
        self.nodes = []

    def put(self, node, cost):
        heapq.heappush(self.nodes, (cost, node))

    def get(self):
        return heapq.heappop(self.nodes)[1]

    def empty(self):
        return len(self.nodes) == 0

def draw_grid():
    for x in range(0, WIDTH, TILESIZE):
        pg.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, TILESIZE):
        pg.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))

def create_pacman_icon():
    """Create a Pacman-like circle icon"""
    icon = pg.Surface((40, 40), pg.SRCALPHA)
    pg.draw.circle(icon, YELLOW, (20, 20), 18)
    # Draw mouth
    pg.draw.polygon(icon, BLACK, [(20, 20), (35, 10), (35, 30)])
    return icon

def create_ghost_icon():
    """Create a ghost-like icon"""
    icon = pg.Surface((40, 40), pg.SRCALPHA)
    # Body
    pg.draw.rect(icon, RED, (8, 12, 24, 20))
    # Head
    pg.draw.circle(icon, RED, (20, 16), 12)
    # Bottom wavy part
    points = [(8, 32), (12, 28), (16, 32), (20, 28), (24, 32), (28, 28), (32, 32), (32, 40), (8, 40)]
    pg.draw.polygon(icon, RED, points)
    # Eyes
    pg.draw.circle(icon, WHITE, (16, 14), 3)
    pg.draw.circle(icon, WHITE, (24, 14), 3)
    pg.draw.circle(icon, BLACK, (16, 14), 1)
    pg.draw.circle(icon, BLACK, (24, 14), 1)
    return icon

def create_arrow_icon(direction):
    """Create arrow icons for path visualization"""
    icon = pg.Surface((30, 30), pg.SRCALPHA)
    # Draw a simple arrow
    center = (15, 15)
    if direction == (1, 0):  # Right
        points = [(25, 15), (15, 10), (15, 20)]
    elif direction == (-1, 0):  # Left
        points = [(5, 15), (15, 10), (15, 20)]
    elif direction == (0, 1):  # Down
        points = [(15, 25), (10, 15), (20, 15)]
    elif direction == (0, -1):  # Up
        points = [(15, 5), (10, 15), (20, 15)]
    elif direction == (1, 1):  # Down-Right
        points = [(20, 20), (10, 15), (15, 10)]
    elif direction == (-1, 1):  # Down-Left
        points = [(8, 20), (18, 15), (13, 10)]
    elif direction == (1, -1):  # Up-Right
        points = [(20, 8), (10, 13), (15, 18)]
    elif direction == (-1, -1):  # Up-Left
        points = [(8, 8), (18, 13), (13, 18)]
    else:
        points = [(15, 10), (20, 15), (15, 20), (10, 15)]
    
    pg.draw.polygon(icon, WHITE, points)
    return icon

def draw_icons():
    ghost_center = (goal.x * TILESIZE + TILESIZE / 2, goal.y * TILESIZE + TILESIZE / 2)
    screen.blit(ghost_img, ghost_img.get_rect(center=ghost_center))
    
    pacman_center = (start.x * TILESIZE + TILESIZE / 2, start.y * TILESIZE + TILESIZE / 2)
    screen.blit(pacman_img, pacman_img.get_rect(center=pacman_center))

def vec2int(v):
    return (int(v.x), int(v.y))

def heuristic(a, b):
    # Manhattan distance * 10 untuk konsistensi dengan cost
    return (abs(a.x - b.x) + abs(a.y - b.y)) * 10

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
        for next in graph.find_neighbors(vec(current)):
            next = vec2int(next)
            next_cost = cost[current] + graph.cost(current, next)
            if next not in cost or next_cost < cost[next]:
                cost[next] = next_cost
                priority = next_cost + heuristic(end, vec(next))  # g(n) + h(n)
                frontier.put(next, priority)
                path[next] = vec(current) - vec(next)
    return path, cost

def dijkstra_search(graph, start, end):
    """
    Dijkstra Search: Uniform Cost Search
    Hanya menggunakan g(n) - cost sebenarnya
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
        for next in graph.find_neighbors(vec(current)):
            next = vec2int(next)
            next_cost = cost[current] + graph.cost(current, next)
            if next not in cost or next_cost < cost[next]:
                cost[next] = next_cost
                priority = next_cost  # Hanya g(n)
                frontier.put(next, priority)
                path[next] = vec(current) - vec(next)
    return path, cost

def greedy_bfs_search(graph, start, end):
    """
    Greedy Best-First Search: Hanya menggunakan heuristic
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
        for next in graph.find_neighbors(vec(current)):
            next = vec2int(next)
            if next not in path:  # Belum dikunjungi
                priority = heuristic(end, vec(next))  # Hanya h(n)
                frontier.put(next, priority)
                path[next] = vec(current) - vec(next)
    
    # Untuk konsistensi dengan fungsi lain, return dummy cost
    cost = {node: 0 for node in path}
    return path, cost

def dfs_search(graph, start, end):
    """
    Depth-First Search: Menggunakan stack (LIFO)
    Tidak optimal tapi akan menemukan jalur jika ada
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
            
        # Shuffle neighbors untuk variasi jalur
        neighbors = list(graph.find_neighbors(vec(current)))
        random.shuffle(neighbors)
        
        for next in neighbors:
            next = vec2int(next)
            if next not in visited and next not in path:
                stack.append(next)
                path[next] = vec(current) - vec(next)
    
    # Return path dengan cost dummy untuk konsistensi
    cost = {node: 0 for node in path}
    return path, cost

def count_path_steps(path, start, goal):
    """Count the number of steps in the path from start to goal"""
    if vec2int(goal) not in path:
        return 0
    
    current = start
    steps = 0
    
    while current != goal and vec2int(current) in path and path[vec2int(current)] is not None:
        steps += 1
        v = path[vec2int(current)]
        current = current + v
        
        # Prevent infinite loops
        if steps > 1000:
            break
    
    return steps

def generate_recursive_backtrack_maze(width, height):
    """
    Recursive Backtracking Algorithm untuk generate maze
    Algoritma yang stabil dan reliable untuk maze generation
    """
    # Mulai dengan semua cell sebagai wall
    walls = set()
    for x in range(width):
        for y in range(height):
            walls.add(vec(x, y))
    
    # Pilih starting cell
    current = vec(1, 1)  # Mulai dari posisi yang aman
    visited = {current}
    walls.discard(current)
    
    # Stack untuk backtracking
    stack = [current]
    
    # Directions untuk maze (hanya 4 arah utama)
    directions = [vec(0, 2), vec(2, 0), vec(0, -2), vec(-2, 0)]
    
    while stack:
        current = stack[-1]
        
        # Cari neighbors yang belum dikunjungi
        unvisited_neighbors = []
        for direction in directions:
            next_cell = current + direction
            if (0 <= next_cell.x < width and 0 <= next_cell.y < height and 
                next_cell not in visited):
                unvisited_neighbors.append(next_cell)
        
        if unvisited_neighbors:
            # Pilih random neighbor
            next_cell = random.choice(unvisited_neighbors)
            
            # Remove wall antara current dan next_cell
            wall_between = current + (next_cell - current) / 2
            walls.discard(next_cell)
            walls.discard(wall_between)
            
            # Mark sebagai visited dan tambah ke stack
            visited.add(next_cell)
            stack.append(next_cell)
        else:
            # Backtrack
            stack.pop()
    
    # Pastikan start dan goal bisa diakses
    start_pos = vec(0, 0)
    goal_pos = vec(width - 1, height - 1)
    walls.discard(start_pos)
    walls.discard(goal_pos)
    
    # Buat jalur dari start ke area yang terbuka
    walls.discard(vec(1, 0))
    walls.discard(vec(0, 1))
    # Dan dari goal ke area yang terbuka  
    walls.discard(vec(width - 2, height - 1))
    walls.discard(vec(width - 1, height - 2))
    
    return list(walls)

def simple_maze_generator(width, height):
    """
    Simple maze generator sebagai alternatif
    Membuat maze dengan pola acak tapi memastikan ada jalur
    """
    walls = []
    
    # Buat border walls
    for x in range(width):
        if x != 0 and x != width - 1:  # Jangan block start dan goal
            if random.random() < 0.3:  # 30% chance untuk wall di top/bottom
                walls.append(vec(x, 0))
            if random.random() < 0.3:
                walls.append(vec(x, height - 1))
    
    for y in range(height):
        if y != 0 and y != height - 1:  # Jangan block start dan goal  
            if random.random() < 0.3:  # 30% chance untuk wall di left/right
                walls.append(vec(0, y))
            if random.random() < 0.3:
                walls.append(vec(width - 1, y))
    
    # Tambahkan random internal walls
    for x in range(1, width - 1):
        for y in range(1, height - 1):
            if random.random() < 0.35:  # 35% chance untuk internal wall
                walls.append(vec(x, y))
    
    return walls

# Create Pacman-themed icons
pacman_img = create_pacman_icon()
ghost_img = create_ghost_icon()

# Create arrow icons
arrows = {}
for dir in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
    arrows[dir] = create_arrow_icon(dir)

g = WeightedGrid(GRIDWIDTH, GRIDHEIGHT)

# Original walls dari maze
original_walls = [
    # Row 0
    (1, 0), (12, 0), (14, 0), (18, 0), (20, 0), (21, 0),
    # Row 1
    (1, 1), (3, 1), (5, 1), (6, 1), (7, 1), (8, 1), (9, 1), (10, 1), (12, 1), (14, 1), (16, 1), (18, 1), (21, 1), (23, 1), (24, 1), (25, 1), (26, 1),
    # Row 2
    (1, 2), (2, 2), (3, 2), (5, 2), (10, 2), (12, 2), (14, 2), (16, 2), (18, 2), (19, 2), (21, 2), (26, 2),
    # Row 3
    (2, 3), (7, 3), (8, 3), (10, 3), (12, 3), (14, 3), (16, 3), (19, 3), (21, 3), (23, 3), (24, 3), (26, 3),
    # Row 4
    (1, 4), (2, 4), (4, 4), (5, 4), (6, 4), (7, 4), (10, 4), (14, 4), (16, 4), (17, 4), (19, 4), (24, 4), (26, 4),
    # Row 5
    (1, 5), (4, 5), (9, 5), (10, 5), (11, 5), (12, 5), (14, 5), (17, 5), (19, 5), (20, 5), (21, 5), (22, 5), (24, 5), (26, 5),
    # Row 6
    (1, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (9, 6), (11, 6), (14, 6), (15, 6), (17, 6), (22, 6), (24, 6), (26, 6),
    # Row 7
    (1, 7), (6, 7), (11, 7), (12, 7), (15, 7), (17, 7), (18, 7), (19, 7), (20, 7), (22, 7), (23, 7), (24, 7), (26, 7),
    # Row 8
    (1, 8), (2, 8), (3, 8), (4, 8), (6, 8), (8, 8), (10, 8), (11, 8), (12, 8), (13, 8), (15, 8), (20, 8), (22, 8), (24, 8), (26, 8),
    # Row 9
    (4, 9), (8, 9), (13, 9), (15, 9), (16, 9), (17, 9), (18, 9), (20, 9), (22, 9), (24, 9), (26, 9), (27, 9),
    # Row 10
    (1, 10), (3, 10), (4, 10), (5, 10), (6, 10), (7, 10), (8, 10), (9, 10), (10, 10), (11, 10), (13, 10), (15, 10), (20, 10), (22, 10), (24, 10),
    # Row 11
    (1, 11), (3, 11), (7, 11), (11, 11), (13, 11), (15, 11), (17, 11), (18, 11), (19, 11), (20, 11), (22, 11), (24, 11), (25, 11), (26, 11),
    # Row 12
    (1, 12), (3, 12), (5, 12), (6, 12), (7, 12), (8, 12), (9, 12), (11, 12), (13, 12), (15, 12), (17, 12), (22, 12), (24, 12), (26, 12),
    # Row 13
    (1, 13), (2, 13), (3, 13), (5, 13), (9, 13), (11, 13), (13, 13), (15, 13), (17, 13), (19, 13), (20, 13), (21, 13), (22, 13), (26, 13),
    # Row 14
    (7, 14), (13, 14), (17, 14), (24, 14)
]

# Set original walls
for wall in original_walls:
    g.walls.append(vec(wall))

# Optional terrain weights (kosongkan jika tidak diperlukan)
terrain = []
for tile in terrain:
    g.weights[tile] = 15

goal = vec(27, 8)
start = vec(0, 0)

# List algoritma untuk pergantian dengan SPACE
algorithms = [a_star_search, dijkstra_search, greedy_bfs_search, dfs_search]
algorithm_names = ["A* Search", "Dijkstra Search", "Greedy BFS", "DFS Search"]
current_algorithm_index = 0
search_type = algorithms[current_algorithm_index]

path, c = search_type(g, goal, start)

running = True
while running:
    clock.tick(FPS)
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_ESCAPE:
                running = False
            if event.key == pg.K_SPACE:
                # Berganti ke algoritma berikutnya secara siklis
                current_algorithm_index = (current_algorithm_index + 1) % len(algorithms)
                search_type = algorithms[current_algorithm_index]
                path, c = search_type(g, goal, start)
                steps = count_path_steps(path, start, goal)
                print(f"Switched to: {algorithm_names[current_algorithm_index]} - Steps: {steps}")
            if event.key == pg.K_g:
                # Generate new maze using Recursive Backtracking algorithm
                print("Generating Recursive Backtrack maze...")
                g.walls.clear()
                backtrack_walls = generate_recursive_backtrack_maze(GRIDWIDTH, GRIDHEIGHT)
                for wall in backtrack_walls:
                    g.walls.append(wall)
                path, c = search_type(g, goal, start)
                steps = count_path_steps(path, start, goal)
                print(f"Generated Recursive Backtrack maze with {len(g.walls)} walls! Steps: {steps}")
            if event.key == pg.K_s:
                # Generate simple random maze
                print("Generating simple maze...")
                g.walls.clear()
                simple_walls = simple_maze_generator(GRIDWIDTH, GRIDHEIGHT)
                for wall in simple_walls:
                    g.walls.append(wall)
                path, c = search_type(g, goal, start)
                steps = count_path_steps(path, start, goal)
                print(f"Generated simple maze with {len(g.walls)} walls! Steps: {steps}")
            if event.key == pg.K_r:
                # Reset to original maze
                g.walls.clear()
                for wall in original_walls:
                    g.walls.append(vec(wall))
                path, c = search_type(g, goal, start)
                steps = count_path_steps(path, start, goal)
                print(f"Reset to original maze! Steps: {steps}")
            if event.key == pg.K_c:
                # Clear all walls
                g.walls.clear()
                path, c = search_type(g, goal, start)
                steps = count_path_steps(path, start, goal)
                print(f"Cleared all walls! Steps: {steps}")
            if event.key == pg.K_m:
                # dump the wall list for saving
                print([(int(loc.x), int(loc.y)) for loc in g.walls])
        if event.type == pg.MOUSEBUTTONDOWN:
            mpos = vec(pg.mouse.get_pos()) // TILESIZE
            if event.button == 1:
                if mpos in g.walls:
                    g.walls.remove(mpos)
                else:
                    g.walls.append(mpos)
            if event.button == 2:
                start = mpos
            if event.button == 3:
                goal = mpos
            path, c = search_type(g, goal, start)

    pg.display.set_caption("Pacman Pathfinding Demo - FPS: {:.2f}".format(clock.get_fps()))
    screen.fill(BACKGROUND)
    
    # Fill explored area with dark blue
    for node in path:
        x, y = node
        rect = pg.Rect(x * TILESIZE, y * TILESIZE, TILESIZE, TILESIZE)
        pg.draw.rect(screen, EXPLORED, rect)
        
    draw_grid()
    g.draw()
    
    # Draw dots on empty spaces (Pacman style)
    for x in range(GRIDWIDTH):
        for y in range(GRIDHEIGHT):
            pos = vec(x, y)
            if pos not in g.walls and pos != start and pos != goal:
                dot_x = x * TILESIZE + TILESIZE // 2
                dot_y = y * TILESIZE + TILESIZE // 2
                pg.draw.circle(screen, WHITE, (dot_x, dot_y), 2)
    
    # draw path from start to goal - Skip first arrow to avoid overlap with start icon
    current = start
    l = 0
    path_steps = 0
    # Move to first step before drawing arrows
    if vec2int(current) in path and path[vec2int(current)] is not None:
        v = path[vec2int(current)]
        if v.length_squared() == 1:
            l += 10
        else:
            l += 14
        current = current + v
        path_steps += 1
    
    # Now draw arrows from second step onwards
    while current != goal and vec2int(current) in path and path[vec2int(current)] is not None:
        v = path[vec2int(current)]
        if v.length_squared() == 1:
            l += 10
        else:
            l += 14
        img = arrows[vec2int(v)]
        x = current.x * TILESIZE + TILESIZE / 2
        y = current.y * TILESIZE + TILESIZE / 2
        r = img.get_rect(center=(x, y))
        screen.blit(img, r)
        # find next in path
        current = current + v
        path_steps += 1
        
    draw_icons()
    
    # Display current algorithm name and metrics
    current_name = algorithm_names[current_algorithm_index]
    draw_text(current_name, 28, YELLOW, WIDTH - 10, HEIGHT - 10, align="bottomright")
    draw_text('Path Length: {}'.format(l), 24, WHITE, WIDTH - 10, HEIGHT - 40, align="bottomright")
    draw_text('Steps: {}'.format(path_steps), 24, WHITE, WIDTH - 10, HEIGHT - 70, align="bottomright")
    
    # Controls with Pacman theme colors
    draw_text('SPACE: Change Algorithm', 18, YELLOW, 10, 10)
    draw_text('G: Generate Recursive Maze', 18, CYAN, 10, 30)
    draw_text('S: Generate Simple Maze', 18, PINK, 10, 50)
    draw_text('R: Reset Original Maze', 18, WHITE, 10, 70)
    draw_text('C: Clear All Walls', 18, LIME, 10, 90)
    draw_text('Mouse: Left=Wall, Middle=Pacman, Right=Ghost', 18, ORANGE, 10, 110)
    
    pg.display.flip()

pg.quit()
