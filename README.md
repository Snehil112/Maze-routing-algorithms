# Maze-routing-algorithms

# Maze Routing Problem Research Paper

## Introduction
The maze routing problem, a classic conundrum in computer science and engineering, poses a significant challenge in finding the optimal path through a maze-like structure. This problem has multifaceted applications across various domains, including circuit design [1], robotics path planning [2], and logistics optimization [3]. At its core, the maze routing problem involves navigating from a starting point to a destination while circumventing obstacles, akin to finding a path through a labyrinth. Over the years, researchers have delved into diverse methodologies to tackle this intricate problem, aiming to devise efficient and effective routing algorithms. 

In VLSI design [4], routing is considered one of the most time-consuming steps. An integrated circuit comprises millions to billions of transistors interconnected to perform desired functions. With the advancement in technology, the complexity of the chip also increases, and routing becomes a crucial stage in the Physical design cycle. Efficiently routing connections between components is crucial for ensuring optimal chip performance, minimizing signal delay, and meeting various design constraints. The maze routing problem arises when designers seek to find the shortest, least congested paths for these connections while navigating through the intricate layout of the chip. 

In [5], C.Y. Lee defined the maze routing problem and explained it as a collection of grids, which are either obstacles or passages to traverse. He developed an algorithm that effectively starts from the source and traverses all possible paths in parallel. It creates a matrix structure where the shortest distance from the source is defined in the grid. This matrix, as shown in Figure 1, is expanded until the algorithm finds the target while avoiding all obstacles in the path.

![Lee's algorithm matrix expansion](/Maze-routing-algorithms/figure1.png)

One of the earliest approaches to maze routing dates back to graph theory techniques, where the maze is modeled as a graph with nodes representing intersections and edges representing pathways. Classic algorithms like Dijkstra's algorithm and A* search algorithm have been adapted to find the shortest path through the maze by exploring the graph's nodes and edges while considering the obstacles. 

Despite significant advancements, the maze routing problem remains a challenging puzzle with ongoing research efforts. In this paper, we aim to compare three popular maze routing algorithms, namely Lee, A*, and Dijkstra, based on space and time complexity. This paper focuses on mazes with orders of thousands. Due to this high order, the complexity of the maze is significant, and the results can be used to determine the realistic time taken to route a real-world problem.
