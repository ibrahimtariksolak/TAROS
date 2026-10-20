"""TDOA trilateration ve RF sinyal modeli birim testleri."""

import math

from taros_sigint.tdoa_localizer import compute_cep, tdoa_localize


class TestTDOALocalization:
    """TDOA konum tahmini."""

    def test_known_position(self):
        sensors = [
            (0, 0, 0),
            (1000, 0, 0),
            (0, 1000, 0),
            (500, 500, 0),
        ]
        true_pos = (2000, 3000, 0)
        c = 2.998e8
        ref_dist = math.sqrt(
            true_pos[0] ** 2 + true_pos[1] ** 2)
        tdoa = []
        for s in sensors[1:]:
            d = math.sqrt(
                (true_pos[0] - s[0]) ** 2
                + (true_pos[1] - s[1]) ** 2)
            tdoa.append((d - ref_dist) / c)

        result = tdoa_localize(sensors, tdoa)
        assert result is not None
        cep = compute_cep(result, true_pos)
        assert cep < 5000.0

    def test_too_few_sensors(self):
        result = tdoa_localize([(0, 0, 0), (100, 0, 0)], [0.001])
        assert result is None

    def test_collocated_sensors_still_returns(self):
        sensors = [
            (0, 0, 0),
            (0.01, 0, 0),
            (0, 0.01, 0),
            (0, 0, 0.01),
        ]
        result = tdoa_localize(sensors, [0.0, 0.0, 0.0])
        assert result is not None

    def test_cep_same_point_zero(self):
        cep = compute_cep((100, 200, 0), (100, 200, 50))
        assert cep < 0.01

    def test_cep_known_distance(self):
        cep = compute_cep((100, 0, 0), (0, 0, 0))
        assert abs(cep - 100.0) < 0.01
