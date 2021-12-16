import numpy as np
import matplotlib.pyplot as plt
import copy

def floodFill(image, sr, sc, newColor):
    m, n = len(image), len(image[0])
    grid=copy.copy(image)
    print("id grid",id(grid))
    print("id image",id(image))

    cur_color=image[sr,sc]
    if cur_color==newColor:
        print("can not perform flood-fill")
        return image
    def bfs(i, j):
        nonlocal grid
        grid[i,j] =newColor
        for (a, b) in [[1, 0], [0, 1], [-1, 0], [0, -1]]:
            if 0 <= i + a < m and 0<=j + b < n and grid[i + a, j + b] !=-1 :
                bfs(i + a, j + b)

    bfs(sr,sc)

    print("id grid", id(grid))
    print("id image", id(image))
    return grid


def show_map(grid):
    color_map = {1: np.array([255, 51, 51]),  # 1 is visited area filled with red
                 0: np.array([0, 102, 204]),  # 0 is free space filled with blue color
                 -1: np.array([192, 192, 192])}  # -1 is obstacle filled with gray color
    y_n=len(grid)
    x_n=len(grid[0])

    data_3d = np.ndarray(shape=(y_n, x_n, 3), dtype=int)
    for i in range(y_n):
        for j in range(x_n):
            data_3d[i,j] = color_map[grid[i,j]]
    plt.imshow(data_3d)



if __name__=="__main__":
    image=np.array([
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 1,-1,-1,-1,-1,-1,-1, 1, 0, 0, 0],
        [0, 0, 0, 1,-1, 1, 1, 1, 1,-1, 1, 0, 0, 0],
        [0, 0, 0, 1,-1, 1, 0, 0, 1,-1, 1, 0, 0, 0],
        [0, 0, 0, 1,-1, 1, 0, 0, 1,-1, 1, 0, 0, 0],
        [0, 0, 0, 1,-1, 1, 1, 1, 1,-1, 1, 0, 0, 0],
        [0, 0, 0, 1,-1,-1,-1,-1,-1,-1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0,-1,-1,-1,-1, 0,-1,-1, 0, 0],
        [0, 0, 0, 0, 0,-1, 0, 0,-1, 0,-1,-1, 0, 0],
        [0, 0, 0, 0, 0,-1, 0, 0,-1, 0,-1,-1, 0, 0],
        [0, 0, 0, 0, 0,-1,-1,-1,-1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    ])

    filled_image = floodFill(image, 2, 4, -1)
    figure= plt.figure('flood fill ', figsize=(10, 10))
    plt.title("flood fill algorithm")
    subfig1 = figure.add_subplot(121)
    plt.title("before flood fill")
    show_map(image)
    subfig2=figure.add_subplot(122)
    plt.title("after flood fill")
    show_map(filled_image)
    # plt.savefig('../image/flood_fill_image.png')
    plt.show()







