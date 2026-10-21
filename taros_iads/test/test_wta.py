"""Silah-hedef atama optimizasyonu birim testleri."""

from taros_iads.wta_optimizer import DefenseUnit, WTAOptimizer


class TestDefenseUnit:
    """Savunma birligi testleri."""

    def test_in_range_can_engage(self):
        """Menzil icindeki hedefe ates edebilmeli."""
        unit = DefenseUnit('alpha', 0.0, 0.0, 3000.0)
        assert unit.can_engage(1000.0, 0.0) is True

    def test_out_of_range_cannot_engage(self):
        """Menzil disindaki hedefe ates edememeli."""
        unit = DefenseUnit('alpha', 0.0, 0.0, 3000.0)
        assert unit.can_engage(5000.0, 0.0) is False

    def test_no_ammo_cannot_engage(self):
        """Munisyonu bitmis birlik ates edememeli."""
        unit = DefenseUnit('alpha', 0.0, 0.0, 3000.0, interceptor_count=0)
        assert unit.can_engage(1000.0, 0.0) is False

    def test_not_operational_cannot_engage(self):
        """Operasyonel olmayan birlik ates edememeli."""
        unit = DefenseUnit('alpha', 0.0, 0.0, 3000.0, operational=False)
        assert unit.can_engage(1000.0, 0.0) is False

    def test_success_probability_close(self):
        """Yakin hedefte basari olasiligi yuksek olmali."""
        unit = DefenseUnit('alpha', 0.0, 0.0, 3000.0)
        prob = unit.success_probability(100.0, 0.0)
        assert prob > 0.9

    def test_success_probability_at_range(self):
        """Menzil sinirinda basari olasiligi dusmeli."""
        unit = DefenseUnit('alpha', 0.0, 0.0, 3000.0)
        prob = unit.success_probability(3000.0, 0.0)
        assert prob < 0.6

    def test_success_probability_out_of_range(self):
        """Menzil disinda basari olasiligi sifir olmali."""
        unit = DefenseUnit('alpha', 0.0, 0.0, 3000.0)
        prob = unit.success_probability(5000.0, 0.0)
        assert prob == 0.0


class TestWTAOptimizer:
    """WTA optimizasyon testleri."""

    def test_single_threat_single_unit(self):
        """Tek tehdit tek birligi atanmali."""
        units = [DefenseUnit('alpha', 0.0, 0.0, 3000.0)]
        opt = WTAOptimizer(units)
        threats = [(1, 1000.0, 0.0, 1)]
        result = opt.assign(threats)
        assert len(result) == 1
        assert result[0][0] == 1
        assert result[0][1] == 'alpha'

    def test_no_units_no_assignments(self):
        """Birlik yoksa atama yapilmamali."""
        opt = WTAOptimizer([])
        threats = [(1, 1000.0, 0.0, 1)]
        result = opt.assign(threats)
        assert len(result) == 0

    def test_out_of_range_not_assigned(self):
        """Menzil disindaki tehdit atanmamali."""
        units = [DefenseUnit('alpha', 0.0, 0.0, 1000.0)]
        opt = WTAOptimizer(units)
        threats = [(1, 5000.0, 0.0, 1)]
        result = opt.assign(threats)
        assert len(result) == 0

    def test_closest_unit_preferred(self):
        """Hedefe yakin birlik tercih edilmeli."""
        units = [
            DefenseUnit('far', 0.0, 0.0, 5000.0),
            DefenseUnit('close', 900.0, 0.0, 5000.0),
        ]
        opt = WTAOptimizer(units)
        threats = [(1, 1000.0, 0.0, 1)]
        result = opt.assign(threats)
        assert result[0][1] == 'close'

    def test_priority_ordering(self):
        """Yuksek oncelikli tehdit once atanmali."""
        units = [DefenseUnit('alpha', 0.0, 0.0, 5000.0)]
        opt = WTAOptimizer(units)
        threats = [
            (1, 1000.0, 0.0, 5),
            (2, 500.0, 0.0, 1),
        ]
        result = opt.assign(threats)
        assert result[0][0] == 2

    def test_coverage_calculation(self):
        """Kapsama orani dogru hesaplanmali."""
        units = [DefenseUnit('alpha', 0.0, 0.0, 3000.0)]
        opt = WTAOptimizer(units)
        assignments = [(1, 'alpha', 0, 1, 0.95)]
        assert opt.compute_coverage(assignments, 2) == 0.5
        assert opt.compute_coverage([], 0) == 1.0
