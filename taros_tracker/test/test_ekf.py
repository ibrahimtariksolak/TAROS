"""
EKF birim testleri.

Tahmin, guncelleme, olcum modelleri ve sayisal kararlilik dogrulanir.
"""

import math

import numpy as np

from taros_tracker.ekf import EKF


class TestEKFPrediction:
    """Sabit hiz modeli tahmin adimi."""

    def test_stationary_stays(self):
        ekf = EKF([100.0, 200.0, 50.0, 0.0, 0.0, 0.0])
        ekf.predict(1.0)
        np.testing.assert_allclose(ekf.position, [100.0, 200.0, 50.0], atol=1e-10)

    def test_constant_velocity(self):
        ekf = EKF([0.0, 0.0, 100.0, 10.0, 20.0, -5.0])
        ekf.predict(2.0)
        np.testing.assert_allclose(ekf.position, [20.0, 40.0, 90.0], atol=1e-10)

    def test_covariance_grows(self):
        ekf = EKF([100.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        P_before = ekf.P.copy()
        ekf.predict(1.0)
        assert np.all(np.diag(ekf.P) >= np.diag(P_before))

    def test_multiple_predicts(self):
        ekf = EKF([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
        for _ in range(10):
            ekf.predict(0.1)
        np.testing.assert_allclose(ekf.position[0], 5.0, atol=1e-6)

    def test_zero_dt(self):
        ekf = EKF([100.0, 200.0, 50.0, 10.0, 20.0, 5.0])
        pos_before = ekf.position.copy()
        ekf.predict(0.0)
        np.testing.assert_allclose(ekf.position, pos_before, atol=1e-10)


class TestEKFRadarUpdate:
    """Radar olcum guncellemesi."""

    def test_range_correction(self):
        ekf = EKF([1000.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        true_range = 1200.0
        meas = [true_range, 0.0, 0.0, 0.0]
        ekf.update_radar(meas)
        assert ekf.position[0] > 1000.0

    def test_azimuth_correction(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        meas_az = math.radians(5.0)
        r = math.sqrt(1000**2 + 100**2)
        meas = [r, meas_az, math.atan2(100, 1000), 0.0]
        ekf.update_radar(meas)
        assert ekf.position[1] > 0

    def test_covariance_shrinks(self):
        ekf = EKF([500.0, 300.0, 100.0, 10.0, -5.0, 0.0])
        P_before_trace = np.trace(ekf.P)
        r = math.sqrt(500**2 + 300**2 + 100**2)
        az = math.atan2(300, 500)
        el = math.atan2(100, math.sqrt(500**2 + 300**2))
        meas = [r, az, el, 0.0]
        ekf.update_radar(meas, R=np.diag([100.0, 0.001, 0.001, 0.5]))
        assert np.trace(ekf.P) < P_before_trace

    def test_multiple_updates_converge(self):
        ekf = EKF([900.0, 400.0, 150.0, 0.0, 0.0, 0.0])
        true_pos = [1000.0, 500.0, 200.0]
        r = math.sqrt(sum(p**2 for p in true_pos))
        az = math.atan2(true_pos[1], true_pos[0])
        el = math.atan2(true_pos[2], math.sqrt(true_pos[0]**2 + true_pos[1]**2))

        for _ in range(30):
            ekf.predict(0.1)
            ekf.update_radar([r, az, el, 0.0], R=np.diag([25.0, 0.0001, 0.0001, 0.1]))

        np.testing.assert_allclose(ekf.position, true_pos, atol=50.0)


class TestEKFBearingUpdate:
    """EO/IR ve akustik yonelim olcum guncellemesi."""

    def test_eoir_azimuth_correction(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        meas_az = math.radians(10.0)
        meas_el = math.atan2(100, 1000)
        ekf.update_eoir([meas_az, meas_el])
        assert ekf.position[1] > 0

    def test_acoustic_update(self):
        ekf = EKF([300.0, 200.0, 50.0, 0.0, 0.0, 0.0])
        az = math.atan2(200, 300)
        el = math.atan2(50, math.sqrt(300**2 + 200**2))
        ekf.update_acoustic([az, el])
        assert math.isfinite(ekf.position[0])

    def test_bearing_only_preserves_range(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        r_before = np.linalg.norm(ekf.position)
        az = math.atan2(0, 1000)
        el = math.atan2(100, 1000)
        ekf.update_eoir([az, el], R=np.diag([1e-4, 1e-4]))
        r_after = np.linalg.norm(ekf.position)
        assert abs(r_after - r_before) / r_before < 0.1


class TestEKFMultiSensorFusion:
    """Coklu sensor fuzyonu."""

    def test_radar_then_eoir_reduces_uncertainty(self):
        ekf = EKF([500.0, 300.0, 100.0, 10.0, 5.0, 0.0])
        r = math.sqrt(500**2 + 300**2 + 100**2)
        az = math.atan2(300, 500)
        el = math.atan2(100, math.sqrt(500**2 + 300**2))

        ekf.update_radar([r, az, el, 0.0])
        trace_after_radar = np.trace(ekf.P)

        ekf.update_eoir([az + 0.001, el - 0.001])
        trace_after_both = np.trace(ekf.P)

        assert trace_after_both < trace_after_radar

    def test_three_sensors_better_than_one(self):
        ekf_single = EKF([500.0, 300.0, 100.0, 0.0, 0.0, 0.0])
        ekf_multi = EKF([500.0, 300.0, 100.0, 0.0, 0.0, 0.0])

        r = math.sqrt(500**2 + 300**2 + 100**2)
        az = math.atan2(300, 500)
        el = math.atan2(100, math.sqrt(500**2 + 300**2))

        ekf_single.predict(0.1)
        ekf_single.update_radar([r, az, el, 0.0])

        ekf_multi.predict(0.1)
        ekf_multi.update_radar([r, az, el, 0.0])
        ekf_multi.update_eoir([az, el])
        ekf_multi.update_acoustic([az, el])

        assert np.trace(ekf_multi.P) < np.trace(ekf_single.P)


class TestEKFMahalanobis:
    """Mahalanobis mesafe hesabi."""

    def test_zero_for_perfect_match(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        r = math.sqrt(1000**2 + 100**2)
        az = math.atan2(0, 1000)
        el = math.atan2(100, 1000)
        d = ekf.mahalanobis([r, az, el, 0.0], 'radar')
        assert d < 1.0

    def test_large_for_outlier(self):
        ekf = EKF([1000.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        d = ekf.mahalanobis([5000.0, math.radians(90), 0.0, 50.0], 'radar')
        assert d > 100.0

    def test_bearing_mahalanobis(self):
        ekf = EKF([500.0, 300.0, 100.0, 0.0, 0.0, 0.0])
        az = math.atan2(300, 500)
        el = math.atan2(100, math.sqrt(500**2 + 300**2))
        d = ekf.mahalanobis([az, el], 'eoir')
        assert d < 5.0


class TestEKFNumericalStability:
    """Sayisal kararlilik testleri."""

    def test_near_origin(self):
        ekf = EKF([0.001, 0.001, 0.001, 0.0, 0.0, 0.0])
        ekf.predict(1.0)
        assert all(math.isfinite(v) for v in ekf.position)

    def test_very_far_target(self):
        ekf = EKF([50000.0, 50000.0, 10000.0, 100.0, 100.0, 0.0])
        ekf.predict(1.0)
        r = np.linalg.norm(ekf.position)
        az = math.atan2(ekf.position[1], ekf.position[0])
        el = math.atan2(ekf.position[2], math.hypot(ekf.position[0], ekf.position[1]))
        ekf.update_radar([r, az, el, 0.0])
        assert all(math.isfinite(v) for v in ekf.position)

    def test_many_predict_update_cycles(self):
        ekf = EKF([1000.0, 500.0, 200.0, 15.0, -10.0, 2.0])
        for i in range(100):
            ekf.predict(0.1)
            pos = ekf.position
            r = np.linalg.norm(pos)
            if r < 1e-3:
                break
            az = math.atan2(pos[1], pos[0])
            el = math.atan2(pos[2], math.hypot(pos[0], pos[1]))
            ekf.update_radar([r + 5.0, az + 0.01, el + 0.01, 0.0])
        assert all(math.isfinite(v) for v in ekf.position)
        assert np.all(np.isfinite(ekf.P))

    def test_covariance_stays_symmetric(self):
        ekf = EKF([1000.0, 500.0, 200.0, 10.0, 5.0, 0.0])
        for _ in range(50):
            ekf.predict(0.1)
            r = np.linalg.norm(ekf.position)
            az = math.atan2(ekf.position[1], ekf.position[0])
            el = math.atan2(ekf.position[2],
                            math.hypot(ekf.position[0], ekf.position[1]))
            ekf.update_radar([r, az, el, 0.0])
        sym_err = np.max(np.abs(ekf.P - ekf.P.T))
        assert sym_err < 1e-6
