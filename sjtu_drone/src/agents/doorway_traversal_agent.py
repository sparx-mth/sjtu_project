import numpy as np
import imageio.v2 as imageio
import yaml
import matplotlib.pyplot as plt
from typing import List, Tuple, Set


class DoorDetector:
    """
    Detect doorway openings in an occupancy grid map (PGM + YAML).

    Assumed PGM values in your map:
        0   = wall (occupied)
        205 = unknown
        254 = free space
    """

    def __init__(self, grid: np.ndarray, resolution: float, origin: Tuple[float, float, float]):
        self.grid = grid
        self.resolution = resolution
        self.origin = origin

        self.height, self.width = grid.shape

        # Pre-classify pixels for faster processing
        self.wall = (grid == 0)
        self.free = (grid == 254)
        self.unknown = (grid == 205)

    @classmethod
    def load(cls, pgm_path: str, yaml_path: str) -> "DoorDetector":
        """Load PGM + YAML and return initialized detector."""
        grid = imageio.imread(pgm_path)

        with open(yaml_path, "r") as f:
            cfg = yaml.safe_load(f)

        resolution = float(cfg["resolution"])
        origin = tuple(cfg["origin"])

        return cls(grid, resolution, origin)

    def detect_doors(self,
                     min_width_m: float = 0.6,
                     max_width_m: float = 1.6) -> List[Tuple[int, int]]:
        """
        Scan all rows and columns to detect doorway center cells.
        """
        min_cells = max(1, int(round(min_width_m / self.resolution)))
        max_cells = int(round(max_width_m / self.resolution))

        doors: Set[Tuple[int, int]] = set()

        # ---------- Vertical scan (opening in horizontal direction) ----------
        for x in range(1, self.width - 1):
            y = 1
            while y < self.height - 1:

                # Move until wall
                if not self.wall[y, x]:
                    y += 1
                    continue

                # Skip continuous wall segment
                while y < self.height - 1 and self.wall[y, x]:
                    y += 1

                gap_start = y

                # Count free space gap
                while y < self.height - 1 and self.free[y, x]:
                    y += 1

                gap_end = y
                gap_len = gap_end - gap_start

                # Check doorway size + walls on both sides
                if min_cells <= gap_len <= max_cells:
                    if self.wall[gap_start - 1, x] and self.wall[gap_end, x]:
                        cy = (gap_start + gap_end - 1) // 2
                        doors.add((x, cy))

        # ---------- Horizontal scan (opening in vertical direction) ----------
        for y in range(1, self.height - 1):
            x = 1
            while x < self.width - 1:

                if not self.wall[y, x]:
                    x += 1
                    continue

                while x < self.width - 1 and self.wall[y, x]:
                    x += 1

                gap_start = x

                while x < self.width - 1 and self.free[y, x]:
                    x += 1

                gap_end = x
                gap_len = gap_end - gap_start

                if min_cells <= gap_len <= max_cells:
                    if self.wall[y, gap_start - 1] and self.wall[y, gap_end]:
                        cx = (gap_start + gap_end - 1) // 2
                        doors.add((cx, y))

        return sorted(doors, key=lambda p: (p[1], p[0]))

    # Optional conversion
    def grid_to_world(self, x: int, y: int) -> Tuple[float, float]:
        ox, oy, _ = self.origin
        return (ox + (x + 0.5) * self.resolution,
                oy + (y + 0.5) * self.resolution)


def show_doors_on_map(grid: np.ndarray, doors: List[Tuple[int, int]]):
    """
    Display the occupancy grid image and overlay detected doorway markers.
    """
    plt.figure(figsize=(12, 6))
    plt.imshow(grid, cmap='gray')

    xs = [d[0] for d in doors]
    ys = [d[1] for d in doors]

    plt.scatter(xs, ys, s=20, marker='x')  # default color
    plt.title(f"Detected Doors: {len(doors)}")
    plt.tight_layout()
    plt.show()


# ----------------------- RUN EXAMPLE -----------------------

if __name__ == "__main__":
    pgm = "/home/user/PycharmProjects/sjtu_project/sjtu_drone/maps/hospital_map_cropped.pgm"
    yaml_file = "/home/user/PycharmProjects/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml"

    detector = DoorDetector.load(pgm, yaml_file)
    doors = detector.detect_doors(min_width_m=0.8, max_width_m=1.0)

    show_doors_on_map(detector.grid, doors)
