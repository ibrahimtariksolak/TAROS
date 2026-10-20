"""Veri iliskilendirme ve iz yonetimi birim testleri."""

import math

import numpy as np

from taros_tracker.data_association import associate, gated_cost_matrix
from taros_tracker.ekf import EKF
from taros_tracker.track_manager import (
    CONFIRMED,
    COASTING,
    DELETED,
    TENTATIVE,
    ManagedTrack,
    TrackManager,
)


class TestGatedCostMatrix:
    """Mahalanobis maliyet matrisi ve kapilama."""

    def test_empty_tracks(self):
        cost, gated = gated_cost_matrix([], [np.array([100, 0, 0, 0])], 'radar')
        assert cost.shape == (0, 1)

    def test_empty_measurements(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        cost, gated = gated_cost_matrix([ekf], [], 'radar')
        assert cost.shape == (1, 0)

    def test_close_measurement_gated_in(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        r = math.sqrt(1000**2 + 100**2)
        az = math.atan2(0, 1000)
        el = math.atan2(100, 1000)
        meas = np.array([r, az, el, 0.0])
        _, gated = gated_cost_matrix([ekf], [meas], 'radar', gate_threshold=50.0)
        assert gated[0, 0]

    def test_far_measurement_gated_out(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        meas = np.array([9000.0, math.radians(90), 0.0, 100.0])
        _, gated = gated_cost_matrix([ekf], [meas], 'radar', gate_threshold=16.0)
        assert not gated[0, 0]


class TestAssociation:
    """Macar algoritmasi iz-olcum eslestirilmesi."""

    def test_single_match(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        r = math.sqrt(1000**2 + 100**2)
        az = math.atan2(0, 1000)
        el = math.atan2(100, 1000)
        meas = [np.array([r, az, el, 0.0])]
        matches, unm_t, unm_m = associate([ekf], meas, 'radar', gate_threshold=50.0)
        assert len(matches) == 1
        assert matches[0] == (0, 0)
        assert len(unm_t) == 0
        assert len(unm_m) == 0

    def test_no_tracks(self):
        meas = [np.array([1000.0, 0.0, 0.0, 0.0])]
        matches, unm_t, unm_m = associate([], meas, 'radar')
        assert len(matches) == 0
        assert len(unm_m) == 1

    def test_no_measurements(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        matches, unm_t, unm_m = associate([ekf], [], 'radar')
        assert len(matches) == 0
        assert len(unm_t) == 1

    def test_two_tracks_two_meas(self):
        ekf1 = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        ekf2 = EKF([0.0, 1000.0, 200.0, 0.0, 0.0, 0.0])

        r1 = math.sqrt(1000**2 + 100**2)
        r2 = math.sqrt(1000**2 + 200**2)
        m1 = np.array([r1, math.atan2(0, 1000), math.atan2(100, 1000), 0.0])
        m2 = np.array([r2, math.atan2(1000, 0), math.atan2(200, 1000), 0.0])

        matches, unm_t, unm_m = associate([ekf1, ekf2], [m1, m2], 'radar', gate_threshold=50.0)
        assert len(matches) == 2
        assert len(unm_t) == 0
        assert len(unm_m) == 0

    def test_unmatched_when_gated_out(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        meas = [np.array([9000.0, math.radians(90), 0.0, 100.0])]
        matches, unm_t, unm_m = associate([ekf], meas, 'radar', gate_threshold=16.0)
        assert len(matches) == 0
        assert len(unm_t) == 1
        assert len(unm_m) == 1

    def test_bearing_association(self):
        ekf = EKF([500.0, 300.0, 100.0, 0.0, 0.0, 0.0])
        az = math.atan2(300, 500)
        el = math.atan2(100, math.sqrt(500**2 + 300**2))
        meas = [np.array([az, el])]
        matches, _, _ = associate([ekf], meas, 'eoir', gate_threshold=50.0)
        assert len(matches) == 1


class TestManagedTrack:
    """Iz yasam dongusu."""

    def test_starts_tentative(self):
        t = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        assert t.status == TENTATIVE

    def test_confirms_after_hits(self):
        t = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        for i in range(3):
            t.record_hit('radar', 'COMMERCIAL_QUAD', float(i))
        assert t.status == CONFIRMED

    def test_coasting_after_timeout(self):
        t = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        for i in range(3):
            t.record_hit('radar', '', float(i))
        t.check_timeout(10.0)
        assert t.status == COASTING

    def test_deleted_after_long_timeout(self):
        t = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        for i in range(3):
            t.record_hit('radar', '', float(i))
        t.check_timeout(6.0)
        assert t.status == COASTING
        t.check_timeout(12.0)
        assert t.status == DELETED

    def test_coasting_recovers_on_hit(self):
        t = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        for i in range(3):
            t.record_hit('radar', '', float(i))
        t.check_timeout(10.0)
        assert t.status == COASTING
        t.record_hit('eoir', '', 10.5)
        assert t.status == CONFIRMED

    def test_classification_voting(self):
        t = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        t.record_hit('radar', 'COMMERCIAL_QUAD', 0.0)
        t.record_hit('eoir', 'COMMERCIAL_QUAD', 0.1)
        t.record_hit('acoustic', 'MILITARY', 0.2)
        assert t.classification == 'COMMERCIAL_QUAD'

    def test_sensor_contributions(self):
        t = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        t.record_hit('radar', '', 0.0)
        t.record_hit('radar', '', 0.1)
        t.record_hit('eoir', '', 0.2)
        contribs = t.sensor_contributions
        assert contribs[0] > contribs[1]
        assert contribs[2] == 0.0

    def test_confidence_multi_sensor_higher(self):
        t_single = ManagedTrack(0, [100, 0, 50, 0, 0, 0], 0.0)
        t_multi = ManagedTrack(1, [100, 0, 50, 0, 0, 0], 0.0)

        for i in range(3):
            t_single.record_hit('radar', '', float(i))
            t_multi.record_hit('radar', '', float(i))
        t_multi.record_hit('eoir', '', 3.0)
        t_multi.record_hit('acoustic', '', 3.1)

        assert t_multi.confidence > t_single.confidence


class TestTrackManager:
    """Iz yoneticisi."""

    def test_initiate_and_list(self):
        mgr = TrackManager()
        mgr.initiate_track([100, 0, 50, 0, 0, 0], 0.0)
        mgr.initiate_track([200, 0, 50, 0, 0, 0], 0.0)
        assert len(mgr.active_tracks) == 2

    def test_deleted_tracks_cleaned(self):
        mgr = TrackManager()
        mgr.initiate_track([100, 0, 50, 0, 0, 0], 0.0)
        mgr.maintain(20.0)
        assert len(mgr.active_tracks) == 0

    def test_predict_all(self):
        mgr = TrackManager()
        t = mgr.initiate_track([100, 0, 50, 10, 0, 0], 0.0)
        mgr.predict_all(1.0)
        assert t.ekf.position[0] > 100.0

    def test_get_ekf_list(self):
        mgr = TrackManager()
        mgr.initiate_track([100, 0, 50, 0, 0, 0], 0.0)
        mgr.initiate_track([200, 0, 50, 0, 0, 0], 0.0)
        ekfs, tracks = mgr.get_ekf_list()
        assert len(ekfs) == 2
        assert len(tracks) == 2
