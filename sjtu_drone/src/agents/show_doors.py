import numpy as np
import imageio.v2 as imageio
import matplotlib.pyplot as plt


def show_doors_on_map(pgm_path: str, doors):
    """
    Display the PGM map and mark every door with a red X.
    (0,0) is assumed to be bottom-left on user's coordinate system.
    """

    # Load map
    grid = imageio.imread(pgm_path)
    h, w = grid.shape

    # Convert door coords from (0,0) bottom-left -> (0,0) top-left
    corrected_doors = [(x, h - 1 - y) for (x, y) in doors]

    xs = [d[0] for d in corrected_doors]
    ys = [d[1] for d in corrected_doors]

    # Display
    plt.figure(figsize=(12, 6))
    plt.imshow(grid, cmap="gray")

    # Mark doors
    plt.scatter(xs, ys, s=50, c='red', marker='x')

    plt.title(f"Doors Marked: {len(doors)}")
    plt.tight_layout()
    plt.show()




# ---------------------- RUN EXAMPLE ----------------------

if __name__ == "__main__":
    pgm_path = "/home/user/PycharmProjects/sjtu_project/sjtu_drone/maps/hospital_map_cropped.pgm"

    # ---------------- INSERT YOUR DOOR LIST HERE ----------------
    doors = [
        (133, 75),
        (256, 75),
        (157, 252),
        (457, 225),
        (157, 299),
        (157, 649),
        (482, 298),
        (249, 475),
        (390, 475),
        (249, 624),
        (390, 624),
        (482, 649),
        (390, 875),
        (249, 875),
        (140, 862),
        (500, 862),
        (93, 996),
        (93, 1156),
        (150, 1175),
        (188, 1250),
        (545, 996),
        (545, 1156),
        (488, 1175),
        (448, 1250)
    ]
    # ------------------------------------------------------------

    show_doors_on_map(pgm_path, doors)
