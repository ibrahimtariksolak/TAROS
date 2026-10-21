"""Arazi modeli, gorus hatti (LOS) ve gorunurluk analizi hesaplamalari."""

import math


class ElevationGrid:
    """Basit 2B yukseklik izgara modeli."""

    def __init__(self, width=1000.0, height=1000.0, resolution=10.0):
        """Yukseklik izgarasini olusturur.

        Args:
            width: Izgara genisligi metre cinsinden.
            height: Izgara yuksekligi metre cinsinden.
            resolution: Hucre boyutu metre cinsinden.
        """
        self._width = width
        self._height = height
        self._resolution = resolution
        self._cols = int(width / resolution)
        self._rows = int(height / resolution)
        self._grid = [[0.0] * self._cols for _ in range(self._rows)]
        self._obstacles = []

    @property
    def resolution(self):
        """Izgara cozunurlugunu dondurur."""
        return self._resolution

    def add_obstacle(self, cx, cy, radius, elevation):
        """Dairesel engel ekler.

        Args:
            cx: Merkez x konumu metre cinsinden.
            cy: Merkez y konumu metre cinsinden.
            radius: Engel yaricapi metre cinsinden.
            elevation: Engel yuksekligi metre cinsinden.
        """
        self._obstacles.append((cx, cy, radius, elevation))
        r0 = max(0, int((cy - radius) / self._resolution))
        r1 = min(self._rows, int((cy + radius) / self._resolution) + 1)
        c0 = max(0, int((cx - radius) / self._resolution))
        c1 = min(self._cols, int((cx + radius) / self._resolution) + 1)

        for r in range(r0, r1):
            for c in range(c0, c1):
                px = (c + 0.5) * self._resolution
                py = (r + 0.5) * self._resolution
                dist = math.hypot(px - cx, py - cy)
                if dist <= radius:
                    self._grid[r][c] = max(self._grid[r][c], elevation)

    def get_elevation(self, x, y):
        """Verilen konumdaki yuksekligi dondurur.

        Args:
            x: X koordinati metre cinsinden.
            y: Y koordinati metre cinsinden.

        Returns:
            Yukseklik degeri metre cinsinden.
        """
        c = int(x / self._resolution)
        r = int(y / self._resolution)
        if 0 <= r < self._rows and 0 <= c < self._cols:
            return self._grid[r][c]
        return 0.0


def check_line_of_sight(grid, observer, target):
    """Gozlemci ile hedef arasinda gorus hatti olup olmadigini kontrol eder.

    Bresenham benzeri ornekleme ile arazi engeli kesisimini arar.

    Args:
        grid: ElevationGrid nesnesi.
        observer: (x, y, z) gozlemci konumu metre cinsinden.
        target: (x, y, z) hedef konumu metre cinsinden.

    Returns:
        True ise gorus hatti acik, False ise engellidir.
    """
    ox, oy, oz = observer
    tx, ty, tz = target

    dist = math.hypot(tx - ox, ty - oy)
    if dist < 1e-6:
        return True

    step = grid.resolution * 0.5
    n_steps = max(int(dist / step), 1)

    for i in range(1, n_steps):
        frac = i / n_steps
        sx = ox + frac * (tx - ox)
        sy = oy + frac * (ty - oy)
        sz = oz + frac * (tz - oz)

        terrain_z = grid.get_elevation(sx, sy)
        if terrain_z > sz:
            return False

    return True


def compute_viewshed(grid, observer, max_range, angle_steps=36):
    """Gozlemci noktasindan gorunur alanlarin haritasini hesaplar.

    Args:
        grid: ElevationGrid nesnesi.
        observer: (x, y, z) gozlemci konumu.
        max_range: Maksimum menzil metre cinsinden.
        angle_steps: Azimut aci adim sayisi.

    Returns:
        Kor nokta acilarinin listesi (radyan cinsinden).
    """
    blind_angles = []
    for i in range(angle_steps):
        angle = 2.0 * math.pi * i / angle_steps
        tx = observer[0] + max_range * math.cos(angle)
        ty = observer[1] + max_range * math.sin(angle)
        tz = 50.0

        if not check_line_of_sight(grid, observer, (tx, ty, tz)):
            blind_angles.append(angle)

    return blind_angles
