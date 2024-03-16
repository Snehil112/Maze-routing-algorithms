import time
import heapq
import matplotlib.pyplot as plt
f = open("maze10.csv" , "r")
total_lines = len(f.readlines())
f.seek(0)
maze = {}
line_count = 0

for x in f:
  maze_line = x.split(",")
  for i in range (len(maze_line)):
    if(maze_line[i] == "w"):
      maze[(line_count,i)] = 0
    elif(maze_line[i] == "c"):
      maze[(line_count,i)] = 1
      if(line_count==0):
        source = (line_count,i)
      elif (line_count == (total_lines-1)):
        target = (line_count,i)
  line_count = line_count+1
print (source)
print (target)
print (maze[source])
print (maze[target])
#print (maze)

def lee_algorithm (source, target,maze):
  cells = []
  rows = max(coord[0] for coord in maze.keys()) + 1
  cols = max(coord[1] for coord in maze.keys()) + 1
  cells.append(source)
  visited_cells = set()
  distance = {source:0}
  while (len(cells) > 0):
    cell = cells.pop()
    if(cell == target):
      return distance[target]
#      print ("Target found after a distance of "+str(distance[target]))
#      return
    for next_cell in [(cell[0] + 1, cell[1]), (cell[0] - 1, cell[1]), (cell[0], cell[1] + 1), (cell[0], cell[1] - 1)]:
      if 0 <= next_cell[0] < rows and 0 <= next_cell[1] < cols and (maze[next_cell] == 1 or maze[next_cell] == 3) and next_cell not in visited_cells:
        visited_cells.add(next_cell)
        distance[next_cell] = distance[cell] + 1
        cells.append(next_cell)
  raise ValueError("No valid path found between source and target cell.")


def heuristic(cell, target):
    return abs(cell[0] - target[0]) + abs(cell[1] - target[1])

def astar_algorithm(source, target, maze):
    if source not in maze or target not in maze:
        raise ValueError("Source or target cell not found in the maze.")

    if maze[source] == 0 or maze[target] == 0:
        raise ValueError("Source or target cell is a blocking cell.")

    visited = set()
    g_score = {source: 0}
    f_score = {source: heuristic(source, target)}
    open_set = [(f_score[source], source)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == target:
            return g_score[target]

        visited.add(current)
        for neighbor in [(current[0] + 1, current[1]), (current[0] - 1, current[1]), (current[0], current[1] + 1), (current[0], current[1] - 1)]:
            if neighbor in visited or neighbor not in maze or maze[neighbor] == 0:
                continue

            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, target)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    raise ValueError("No valid path found between source and target cell.")


def dijkstra_algorithm(source, target, maze):
    if source not in maze or target not in maze:
        raise ValueError("Source or target cell not found in the maze.")

    if maze[source] == 0 or maze[target] == 0:
        raise ValueError("Source or target cell is a blocking cell.")

    # Initialize distances with infinity for all cells except the source cell
    distances = {cell: float('inf') for cell in maze}
    distances[source] = 0

    # Priority queue to keep track of the next cell to visit
    pq = [(0, source)]

    while pq:
        dist_to_current, current = heapq.heappop(pq)

        # If current cell is the target, return its distance
        if current == target:
            return dist_to_current

        # Visit neighboring cells
        for neighbor in [(current[0] + 1, current[1]), (current[0] - 1, current[1]),
                         (current[0], current[1] + 1), (current[0], current[1] - 1)]:
            if neighbor in maze and maze[neighbor] == 1:
                # Calculate the tentative distance to the neighbor
                dist_to_neighbor = dist_to_current + 1

                # Update the distance if the tentative distance is shorter than the current distance
                if dist_to_neighbor < distances[neighbor]:
                    distances[neighbor] = dist_to_neighbor
                    heapq.heappush(pq, (dist_to_neighbor, neighbor))

    raise ValueError("No valid path found between source and target cell.")

def compare_algorithms(source, target, maze):
    start_time = time.time()
    shortest_distance_lee = lee_algorithm(source, target, maze)
    lee_time = time.time() - start_time

    start_time = time.time()
    shortest_distance_dijkstra = dijkstra_algorithm(source, target, maze)
    dijkstra_time = time.time() - start_time

    start_time = time.time()
    shortest_distance_a_star = astar_algorithm(source, target, maze)
    a_star_time = time.time() - start_time

    return {"Lee's Algorithm": lee_time, "Dijkstra's Algorithm": dijkstra_time, "A* Algorithm": a_star_time}

# Define your maze and source/target cells here

results = compare_algorithms(source, target, maze)
print("Time taken by each algorithm:")
for algo, time_taken in results.items():
    print(f"{algo}: {time_taken} seconds")

# Plotting
plt.bar(results.keys(), results.values())
plt.xlabel('Algorithm')
plt.ylabel('Time taken (seconds)')
plt.title('Time taken by each algorithm')
plt.show()


