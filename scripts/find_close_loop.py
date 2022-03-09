import numpy as np

grid_map=np.array([
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0],
                [0,-1, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0],
                [0,-1, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0],
                [0,-1, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0],
                [0,-1, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0],
                [0,-1, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0],
                [0,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
])
print(grid_map)
def numIslands(grid):
    m, n = len(grid), len(grid[0])
    grid2=grid

    def bfs(i, j):
        nonlocal grid
        grid[i,j] = 0
        for (a, b) in [[1, 0], [0, 1], [-1, 0], [0, -1]]:
            if i + a >= 0 and i + a < m and j + b >= 0 and j + b < n and grid[i + a,j + b] == -1:
                bfs(i + a, j + b)

    count = 0
    for i in range(m):
        for j in range(n):
            if grid[i,j] == -1:
                start_point=[i,j]
                count += 1
                bfs(i, j)
    return count
print(numIslands(grid_map))