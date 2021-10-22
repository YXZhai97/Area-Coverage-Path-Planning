import collections
from typing import List

import numpy as np


def nearest_beta(grid: List[List[int]], state: List[int]) -> List[int]:
    rows, cols = len(grid), len(grid[0])
    visit = set()
    beta_neighbour = []
    grid_length = 2
    rs = 8

    def get_center(row, col):
        x_center = grid_length / 2 + row * grid_length
        y_center = grid_length / 2 + col * grid_length
        cell_center = np.array([x_center, y_center])
        return cell_center

    def bfs(r, c):
        q = collections.deque()
        visit.add((r, c))
        q.append((r, c))
        min_distance = rs

        while q:
            row, col = q.popleft()
            center = get_center(row, col)
            distance = np.linalg.norm(state - center)
            if distance < min_distance:
                min_distance = distance
                nearest_neighbour = center

            directions = [[1, 0], [-1, 0], [0, 1], [0, -1]]
            for dr, dc in directions:
                r, c = row + dr, col + dc
                if (r in range(rows) and
                        c in range(cols) and
                        grid[r][c] == -1 and
                        (r, c) not in visit):
                    q.append((r, c))
                    visit.add((r, c))

        return nearest_neighbour

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == -1 and (r, c) not in visit:
                beta_neighbour.append(bfs(r, c))



    return beta_neighbour


grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, -1, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, -1, 0, 0, 0, 0, 0, -1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, -1, 0, 0],
        [0, 0, 0, 0, 0, 0, -1, -1, 0, 0],
        [0, -1, -1, 0, 0, 0, 0, -1, 0, 0],
        [0, -1, -1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

print(nearest_beta(grid, [10, 10]))
