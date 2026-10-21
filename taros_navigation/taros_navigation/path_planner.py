"""2B izgara uzerinde A-yildiz yol planlama modulu."""

import heapq
import math


class GridMap:
    """Engel bilgisi tutan 2B izgara haritasi."""

    def __init__(self, width, height, resolution=1.0):
        """Izgara haritasini olusturur.

        Args:
            width: Harita genisligi metre cinsinden.
            height: Harita yuksekligi metre cinsinden.
            resolution: Hucre boyutu metre cinsinden.
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.cols = int(width / resolution)
        self.rows = int(height / resolution)
        self._blocked = set()

    def add_obstacle(self, cx, cy, radius):
        """Dairesel engel ekler.

        Args:
            cx: Merkez x koordinati.
            cy: Merkez y koordinati.
            radius: Engel yaricapi metre cinsinden.
        """
        r0 = max(0, int((cy - radius) / self.resolution))
        r1 = min(self.rows, int((cy + radius) / self.resolution) + 1)
        c0 = max(0, int((cx - radius) / self.resolution))
        c1 = min(self.cols, int((cx + radius) / self.resolution) + 1)

        for r in range(r0, r1):
            for c in range(c0, c1):
                px = (c + 0.5) * self.resolution
                py = (r + 0.5) * self.resolution
                if math.hypot(px - cx, py - cy) <= radius:
                    self._blocked.add((r, c))

    def is_free(self, row, col):
        """Hucrenin bos olup olmadigini dondurur."""
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return False
        return (row, col) not in self._blocked

    def world_to_grid(self, x, y):
        """Dunya koordinatlarindan izgara indeksine cevirir."""
        return int(y / self.resolution), int(x / self.resolution)

    def grid_to_world(self, row, col):
        """Izgara indeksinden dunya koordinatlarina cevirir."""
        return (col + 0.5) * self.resolution, (row + 0.5) * self.resolution


def plan_path(grid_map, start_xy, goal_xy):
    """A-yildiz algoritmasi ile yol planlar.

    Args:
        grid_map: GridMap nesnesi.
        start_xy: (x, y) baslangic konumu dunya koordinatlarinda.
        goal_xy: (x, y) hedef konumu dunya koordinatlarinda.

    Returns:
        Dunya koordinatlarinda (x, y) tuplelarindan olusan yol listesi,
        yol bulunamazsa bos liste dondurur.
    """
    sr, sc = grid_map.world_to_grid(start_xy[0], start_xy[1])
    gr, gc = grid_map.world_to_grid(goal_xy[0], goal_xy[1])

    if not grid_map.is_free(sr, sc) or not grid_map.is_free(gr, gc):
        return []

    def heuristic(r, c):
        return math.hypot(r - gr, c - gc)

    open_set = [(heuristic(sr, sc), 0.0, sr, sc)]
    came_from = {}
    g_score = {(sr, sc): 0.0}

    neighbors = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
    ]

    while open_set:
        _, cost, cr, cc = heapq.heappop(open_set)

        if cr == gr and cc == gc:
            path = []
            node = (gr, gc)
            while node in came_from:
                wx, wy = grid_map.grid_to_world(node[0], node[1])
                path.append((wx, wy))
                node = came_from[node]
            wx, wy = grid_map.grid_to_world(sr, sc)
            path.append((wx, wy))
            path.reverse()
            return path

        if cost > g_score.get((cr, cc), float('inf')):
            continue

        for dr, dc, move_cost in neighbors:
            nr, nc = cr + dr, cc + dc
            if not grid_map.is_free(nr, nc):
                continue
            new_g = cost + move_cost * grid_map.resolution
            if new_g < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = new_g
                came_from[(nr, nc)] = (cr, cc)
                f = new_g + heuristic(nr, nc) * grid_map.resolution
                heapq.heappush(open_set, (f, new_g, nr, nc))

    return []


def direct_path(start_xy, goal_xy, step_size=1.0):
    """Engelsiz ortamda dogrudan yol olusturur.

    Args:
        start_xy: (x, y) baslangic konumu.
        goal_xy: (x, y) hedef konumu.
        step_size: Adim buyuklugu metre cinsinden.

    Returns:
        (x, y) tuplelarindan olusan yol listesi.
    """
    dist = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1])
    if dist < 1e-6:
        return [start_xy]

    n_steps = max(int(dist / step_size), 1)
    path = []
    for i in range(n_steps + 1):
        frac = i / n_steps
        x = start_xy[0] + frac * (goal_xy[0] - start_xy[0])
        y = start_xy[1] + frac * (goal_xy[1] - start_xy[1])
        path.append((x, y))
    return path
