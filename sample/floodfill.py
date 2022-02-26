def flood_fill(image, sr, sc, newColor):
    """

    Args:
        image: The original image matrix
        sr: The row number of the seed point
        sc: The colum number of the seed point
        newColor: The filling color

    Returns: An image filled with the newColor

    """
    m, n = len(image), len(image[0])
    grid = image
    cur_color = image[sr, sc]
    if cur_color == newColor:
        print("can not perform flood-fill")
        return image

    def bfs(i, j):
        nonlocal grid
        grid[i, j] = newColor
        for (a, b) in [[1, 0], [0, 1], [-1, 0], [0, -1]]:
            if 0 <= i + a < m and 0 <= j + b < n and grid[i + a, j + b] != -1:
                bfs(i + a, j + b)
    bfs(sr, sc)
    return grid



