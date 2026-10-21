"""Arazi modeli gorus hatti ve gorunurluk birim testleri."""

import math

from taros_terrain.terrain_models import (
    ElevationGrid,
    check_line_of_sight,
    compute_viewshed,
)


class TestElevationGrid:
    """Yukseklik izgara modeli testleri."""

    def test_empty_grid_zero_elevation(self):
        """Bos izgarada yukseklik sifir olmali."""
        grid = ElevationGrid(100.0, 100.0, 10.0)
        assert grid.get_elevation(50.0, 50.0) == 0.0

    def test_obstacle_raises_elevation(self):
        """Engel eklendikten sonra yukseklik artmali."""
        grid = ElevationGrid(200.0, 200.0, 5.0)
        grid.add_obstacle(100.0, 100.0, 20.0, 50.0)
        elev = grid.get_elevation(100.0, 100.0)
        assert elev == 50.0

    def test_outside_obstacle_zero(self):
        """Engel disinda yukseklik sifir olmali."""
        grid = ElevationGrid(200.0, 200.0, 5.0)
        grid.add_obstacle(100.0, 100.0, 10.0, 50.0)
        elev = grid.get_elevation(5.0, 5.0)
        assert elev == 0.0


class TestLineOfSight:
    """Gorus hatti hesaplama testleri."""

    def test_clear_los_flat_terrain(self):
        """Duz arazide gorus hatti acik olmali."""
        grid = ElevationGrid(500.0, 500.0, 5.0)
        observer = (50.0, 50.0, 10.0)
        target = (400.0, 400.0, 10.0)
        assert check_line_of_sight(grid, observer, target) is True

    def test_blocked_los_by_hill(self):
        """Tepe araya girdiginde gorus hatti engellenmeli."""
        grid = ElevationGrid(500.0, 500.0, 5.0)
        grid.add_obstacle(250.0, 250.0, 30.0, 100.0)
        observer = (50.0, 250.0, 10.0)
        target = (450.0, 250.0, 10.0)
        assert check_line_of_sight(grid, observer, target) is False

    def test_los_over_hill_high_altitude(self):
        """Yuksek irtifadaki hedef tepenin uzerinden gorunmeli."""
        grid = ElevationGrid(500.0, 500.0, 5.0)
        grid.add_obstacle(250.0, 250.0, 30.0, 50.0)
        observer = (50.0, 250.0, 10.0)
        target = (450.0, 250.0, 200.0)
        assert check_line_of_sight(grid, observer, target) is True

    def test_same_point_always_visible(self):
        """Ayni noktada gorus hatti her zaman acik olmali."""
        grid = ElevationGrid(100.0, 100.0, 5.0)
        point = (50.0, 50.0, 10.0)
        assert check_line_of_sight(grid, point, point) is True


class TestViewshed:
    """Gorunurluk analizi testleri."""

    def test_flat_terrain_no_blind_zones(self):
        """Duz arazide kor alan olmamali."""
        grid = ElevationGrid(1000.0, 1000.0, 10.0)
        observer = (500.0, 500.0, 15.0)
        blind = compute_viewshed(grid, observer, 400.0)
        assert len(blind) == 0

    def test_surrounded_by_hills(self):
        """Her yonde tepe varsa kor alan olmali."""
        grid = ElevationGrid(1000.0, 1000.0, 10.0)
        for angle_i in range(12):
            a = 2.0 * math.pi * angle_i / 12
            cx = 500.0 + 100.0 * math.cos(a)
            cy = 500.0 + 100.0 * math.sin(a)
            grid.add_obstacle(cx, cy, 40.0, 80.0)
        observer = (500.0, 500.0, 5.0)
        blind = compute_viewshed(grid, observer, 400.0)
        assert len(blind) > 0
