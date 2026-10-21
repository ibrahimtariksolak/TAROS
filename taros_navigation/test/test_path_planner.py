"""Yol planlama birim testleri."""

import math

from taros_navigation.path_planner import (
    GridMap,
    direct_path,
    plan_path,
)


class TestGridMap:
    """Izgara haritasi testleri."""

    def test_empty_map_all_free(self):
        """Bos haritada tum hucreler serbest olmali."""
        gm = GridMap(100.0, 100.0, 10.0)
        assert gm.is_free(5, 5) is True

    def test_obstacle_blocks_cell(self):
        """Engel eklenen hucre bloke olmali."""
        gm = GridMap(100.0, 100.0, 5.0)
        gm.add_obstacle(50.0, 50.0, 10.0)
        r, c = gm.world_to_grid(50.0, 50.0)
        assert gm.is_free(r, c) is False

    def test_out_of_bounds_not_free(self):
        """Sinir disi hucreler serbest olmamali."""
        gm = GridMap(100.0, 100.0, 10.0)
        assert gm.is_free(-1, 0) is False
        assert gm.is_free(0, 100) is False

    def test_world_grid_roundtrip(self):
        """Dunya-izgara-dunya cevrimi tutarli olmali."""
        gm = GridMap(100.0, 100.0, 10.0)
        r, c = gm.world_to_grid(55.0, 35.0)
        wx, wy = gm.grid_to_world(r, c)
        assert abs(wx - 55.0) < 10.0
        assert abs(wy - 35.0) < 10.0


class TestPlanPath:
    """A-yildiz yol planlama testleri."""

    def test_straight_path_no_obstacle(self):
        """Engelsiz ortamda yol bulunmali."""
        gm = GridMap(100.0, 100.0, 5.0)
        path = plan_path(gm, (10.0, 10.0), (90.0, 90.0))
        assert len(path) > 0
        assert math.hypot(
            path[-1][0] - 90.0, path[-1][1] - 90.0) < 10.0

    def test_path_around_obstacle(self):
        """Engeli dolanarak yol bulunmali."""
        gm = GridMap(100.0, 100.0, 5.0)
        gm.add_obstacle(50.0, 50.0, 15.0)
        path = plan_path(gm, (10.0, 50.0), (90.0, 50.0))
        assert len(path) > 0

    def test_blocked_start(self):
        """Baslangic engelde ise bos yol donmeli."""
        gm = GridMap(100.0, 100.0, 5.0)
        gm.add_obstacle(10.0, 10.0, 15.0)
        path = plan_path(gm, (10.0, 10.0), (90.0, 90.0))
        assert path == []

    def test_same_start_goal(self):
        """Baslangic ve hedef ayni ise tek noktali yol donmeli."""
        gm = GridMap(100.0, 100.0, 5.0)
        path = plan_path(gm, (50.0, 50.0), (50.0, 50.0))
        assert len(path) >= 1


class TestDirectPath:
    """Dogrudan yol olusturma testleri."""

    def test_basic_direct_path(self):
        """Dogrudan yol baslangic ve bitiste olmali."""
        path = direct_path((0.0, 0.0), (10.0, 0.0), step_size=2.0)
        assert len(path) > 2
        assert abs(path[0][0]) < 0.01
        assert abs(path[-1][0] - 10.0) < 0.01

    def test_zero_distance(self):
        """Ayni noktada tek adimli yol donmeli."""
        path = direct_path((5.0, 5.0), (5.0, 5.0))
        assert len(path) == 1
