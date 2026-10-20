"""
sensor_models.py birim testleri.

Fizik hesaplamalarinin dogru sonuc verdigini, sinir kosullarinin patlamadan
islendigini ve meteorolojik degradasyonun beklenen yonde etkiledigini dogrular.
"""

import math

from taros_sensor.sensor_models import (
    acoustic_doa_noise,
    acoustic_snr,
    azimuth_elevation,
    db_to_linear,
    distance_3d,
    doppler_velocity,
    eoir_max_range,
    eoir_pixel_from_bearing,
    generate_frequency_spectrum,
    linear_to_db,
    radar_angle_noise,
    radar_range_noise,
    radar_snr,
    radar_system_constant,
    rain_attenuation_db,
    swerling_i_pd,
)


class TestDbConversions:
    """dB <-> lineer donusumleri."""

    def test_roundtrip(self):
        assert abs(db_to_linear(linear_to_db(100.0)) - 100.0) < 1e-6

    def test_zero_db(self):
        assert abs(db_to_linear(0.0) - 1.0) < 1e-9

    def test_negative_linear(self):
        assert linear_to_db(0) == -999.0
        assert linear_to_db(-1) == -999.0

    def test_ten_db(self):
        assert abs(db_to_linear(10.0) - 10.0) < 1e-6

    def test_twenty_db(self):
        assert abs(db_to_linear(20.0) - 100.0) < 1e-4


class TestRadarEquation:
    """Radar SNR ve tespit modeli."""

    def test_system_constant_positive(self):
        k = radar_system_constant()
        assert k > 0

    def test_snr_decreases_with_range(self):
        k = radar_system_constant()
        snr_near = radar_snr(1000.0, -10.0, k)
        snr_far = radar_snr(5000.0, -10.0, k)
        assert snr_near > snr_far

    def test_snr_increases_with_rcs(self):
        k = radar_system_constant()
        snr_small = radar_snr(3000.0, -20.0, k)
        snr_large = radar_snr(3000.0, 0.0, k)
        assert snr_large > snr_small

    def test_snr_range_one_meter_clamp(self):
        k = radar_system_constant()
        snr = radar_snr(0.5, -10.0, k)
        assert math.isfinite(snr)

    def test_rain_reduces_snr(self):
        k = radar_system_constant()
        snr_dry = radar_snr(5000.0, -10.0, k, rain_rate_mmh=0.0)
        snr_rain = radar_snr(5000.0, -10.0, k, rain_rate_mmh=20.0)
        assert snr_dry > snr_rain

    def test_rain_attenuation_zero_rain(self):
        assert rain_attenuation_db(0.0, 5000.0) == 0.0

    def test_rain_attenuation_increases(self):
        a5 = rain_attenuation_db(5.0, 5000.0)
        a20 = rain_attenuation_db(20.0, 5000.0)
        assert a20 > a5 > 0


class TestSwerlingDetection:
    """Swerling-I tespit olasiligi."""

    def test_high_snr_detects(self):
        assert swerling_i_pd(30.0, 13.0) > 0.99

    def test_low_snr_misses(self):
        assert swerling_i_pd(-10.0, 13.0) < 0.01

    def test_at_threshold(self):
        pd = swerling_i_pd(13.0, 13.0)
        assert 0.3 < pd < 0.7

    def test_clamp_very_low(self):
        assert swerling_i_pd(-50.0) == 0.0

    def test_clamp_very_high(self):
        assert swerling_i_pd(50.0) == 1.0


class TestRadarNoise:
    """Radar olcum gurultusu."""

    def test_range_noise_decreases_with_snr(self):
        n_low = radar_range_noise(15.0, 1.0)
        n_high = radar_range_noise(15.0, 100.0)
        assert n_low > n_high

    def test_angle_noise_decreases_with_snr(self):
        n_low = radar_angle_noise(math.radians(2.0), 1.0)
        n_high = radar_angle_noise(math.radians(2.0), 100.0)
        assert n_low > n_high

    def test_very_low_snr_clamped(self):
        n = radar_range_noise(15.0, 0.001)
        assert math.isfinite(n)


class TestDoppler:
    """Radyal hiz hesabi."""

    def test_approaching_target(self):
        vel = doppler_velocity(
            (-10.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0),
        )
        assert vel < 0

    def test_receding_target(self):
        vel = doppler_velocity(
            (10.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0),
        )
        assert vel > 0

    def test_tangential_target(self):
        vel = doppler_velocity(
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0),
        )
        assert abs(vel) < 0.1

    def test_zero_distance(self):
        vel = doppler_velocity((10.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        assert vel == 0.0


class TestGeometry:
    """Geometri yardimcilari."""

    def test_azimuth_elevation_north(self):
        az, el = azimuth_elevation((0, 0, 0), (100, 0, 0))
        assert abs(az) < 0.01
        assert abs(el) < 0.01

    def test_azimuth_elevation_up(self):
        _, el = azimuth_elevation((0, 0, 0), (0, 0, 100))
        assert abs(el - math.pi / 2) < 0.01

    def test_distance_3d_basic(self):
        assert abs(distance_3d((0, 0, 0), (3, 4, 0)) - 5.0) < 1e-6

    def test_distance_3d_same_point(self):
        assert distance_3d((1, 2, 3), (1, 2, 3)) == 0.0


class TestEoirModel:
    """EO/IR kamera modeli."""

    def test_clear_day_full_range(self):
        r = eoir_max_range(5000.0, 10000.0, 0.0, False)
        assert abs(r - 5000.0) < 1.0

    def test_fog_reduces_range(self):
        r_clear = eoir_max_range(5000.0, 10000.0, 0.0, False)
        r_fog = eoir_max_range(5000.0, 2000.0, 0.0, False)
        assert r_fog < r_clear

    def test_night_reduces_range(self):
        r_day = eoir_max_range(5000.0, 10000.0, 0.0, False)
        r_night = eoir_max_range(5000.0, 10000.0, 0.0, True)
        assert r_night < r_day

    def test_rain_reduces_range(self):
        r_dry = eoir_max_range(5000.0, 10000.0, 0.0, False)
        r_wet = eoir_max_range(5000.0, 10000.0, 30.0, False)
        assert r_wet < r_dry

    def test_pixel_center_of_fov(self):
        result = eoir_pixel_from_bearing(0.0, 0.0, 0.0, 0.0, math.radians(30), math.radians(22.5))
        assert result is not None
        px, py = result
        assert abs(px - 0.5) < 0.01
        assert abs(py - 0.5) < 0.01

    def test_pixel_outside_fov(self):
        result = eoir_pixel_from_bearing(
            math.radians(90), 0.0, 0.0, 0.0, math.radians(30), math.radians(22.5))
        assert result is None


class TestAcousticModel:
    """Akustik sensor modeli."""

    def test_snr_decreases_with_range(self):
        snr_near = acoustic_snr(100.0, 75.0)
        snr_far = acoustic_snr(500.0, 75.0)
        assert snr_near > snr_far

    def test_wind_reduces_snr(self):
        snr_calm = acoustic_snr(200.0, 75.0, 40.0, 0.0)
        snr_windy = acoustic_snr(200.0, 75.0, 40.0, 15.0)
        assert snr_calm > snr_windy

    def test_doa_noise_decreases_with_snr(self):
        n_low = acoustic_doa_noise(1.0, 1.0, 200.0)
        n_high = acoustic_doa_noise(100.0, 1.0, 200.0)
        assert n_low > n_high

    def test_doa_noise_finite_at_low_snr(self):
        n = acoustic_doa_noise(0.001, 1.0, 200.0)
        assert math.isfinite(n)

    def test_spectrum_length(self):
        spec = generate_frequency_spectrum('COMMERCIAL_QUAD', 20.0, num_bins=16)
        assert len(spec) == 16

    def test_spectrum_values_finite(self):
        spec = generate_frequency_spectrum('MILITARY', 15.0)
        assert all(math.isfinite(v) for v in spec)
