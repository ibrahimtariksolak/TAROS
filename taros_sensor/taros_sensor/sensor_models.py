"""Sensor fizik modelleri — ROS'tan bagimsiz, birim-testlenebilir fonksiyonlar."""

import math
import random

SPEED_OF_SOUND = 343.0  # m/s @ 20 C
BOLTZMANN_K = 1.380649e-23
SPEED_OF_LIGHT = 2.998e8

RCS_DBSM = {
    'COMMERCIAL_QUAD': -10.0,
    'COMMERCIAL_FIXEDWING': -5.0,
    'MILITARY': -3.0,
    'UNKNOWN': -8.0,
}

SOURCE_SPL_DB = {
    'COMMERCIAL_QUAD': 75.0,
    'COMMERCIAL_FIXEDWING': 70.0,
    'MILITARY': 80.0,
    'UNKNOWN': 72.0,
}

DOMINANT_FREQ_HZ = {
    'COMMERCIAL_QUAD': 220.0,
    'COMMERCIAL_FIXEDWING': 150.0,
    'MILITARY': 180.0,
    'UNKNOWN': 200.0,
}


def db_to_linear(db):
    """Desibeli lineer degere donusturur."""
    return 10.0 ** (db / 10.0)


def linear_to_db(linear):
    """Lineer degeri desibele donusturur."""
    if linear <= 0:
        return -999.0
    return 10.0 * math.log10(linear)


def radar_system_constant(
    transmit_power_dbw=50.0,
    antenna_gain_db=35.0,
    frequency_ghz=9.5,
    noise_figure_db=3.0,
    bandwidth_hz=1e6,
    system_loss_db=5.0,
):
    """Radar denklemi sabit terimlerini hesaplar [dB]."""
    wavelength = SPEED_OF_LIGHT / (frequency_ghz * 1e9)
    k_db = linear_to_db(BOLTZMANN_K)
    t_db = linear_to_db(290.0)
    bw_db = linear_to_db(bandwidth_hz)
    four_pi_cubed_db = linear_to_db((4.0 * math.pi) ** 3)
    wavelength_sq_db = linear_to_db(wavelength ** 2)

    return (
        transmit_power_dbw
        + 2.0 * antenna_gain_db
        + wavelength_sq_db
        - four_pi_cubed_db
        - k_db
        - t_db
        - bw_db
        - noise_figure_db
        - system_loss_db
    )


def radar_snr(range_m, rcs_dbsm, k_sys, rain_rate_mmh=0.0):
    """Hedef SNR'ini hesaplar [dB]."""
    if range_m < 1.0:
        range_m = 1.0
    range_loss_db = 40.0 * math.log10(range_m)
    rain_atten_db = rain_attenuation_db(rain_rate_mmh, range_m)
    return k_sys + rcs_dbsm - range_loss_db - rain_atten_db


def rain_attenuation_db(rain_rate_mmh, range_m):
    """X-band yagmur zayiflamasi [dB]. ITU-R P.838 basitlestirilmis."""
    if rain_rate_mmh <= 0:
        return 0.0
    specific_atten = 0.01 * (rain_rate_mmh ** 1.21)
    return specific_atten * (range_m / 1000.0)


def swerling_i_pd(snr_db, threshold_db=13.0):
    """Swerling-I hedef icin tespit olasiligi."""
    snr_excess = snr_db - threshold_db
    if snr_excess < -20.0:
        return 0.0
    if snr_excess > 20.0:
        return 1.0
    return 1.0 / (1.0 + math.exp(-0.8 * snr_excess))


def radar_range_noise(range_resolution_m, snr_linear):
    """Menzil olcum gurultusu standart sapmasi [m]."""
    if snr_linear < 0.1:
        snr_linear = 0.1
    return range_resolution_m / (2.0 * math.sqrt(2.0 * snr_linear))


def radar_angle_noise(beamwidth_rad, snr_linear):
    """Aci olcum gurultusu standart sapmasi [rad]."""
    if snr_linear < 0.1:
        snr_linear = 0.1
    return beamwidth_rad / (2.0 * math.sqrt(2.0 * snr_linear))


def doppler_velocity(target_velocity, radar_position, target_position):
    """Radyal hiz bileseni [m/s]."""
    dx = target_position[0] - radar_position[0]
    dy = target_position[1] - radar_position[1]
    dz = target_position[2] - radar_position[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-6:
        return 0.0
    ux, uy, uz = dx / dist, dy / dist, dz / dist
    return target_velocity[0] * ux + target_velocity[1] * uy + target_velocity[2] * uz


def azimuth_elevation(origin, target):
    """Gozlemciden hedefe azimut ve yonelim acisi [rad]."""
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    dz = target[2] - origin[2]
    horiz = math.sqrt(dx * dx + dy * dy)
    az = math.atan2(dy, dx)
    el = math.atan2(dz, horiz) if horiz > 1e-6 else (math.pi / 2.0 if dz > 0 else -math.pi / 2.0)
    return az, el


def distance_3d(a, b):
    """Iki nokta arasi Oklid mesafesi."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def eoir_max_range(base_range, visibility_m, rain_rate, is_night):
    """Hava kosullarina bagli EO/IR etkili menzil [m]."""
    vis_factor = min(visibility_m / 10000.0, 1.0)
    rain_factor = max(1.0 - rain_rate / 50.0, 0.2)
    night_factor = 0.6 if is_night else 1.0
    return base_range * vis_factor * rain_factor * night_factor


def eoir_pixel_from_bearing(azimuth_rad, elevation_rad, cam_az, cam_el, h_fov, v_fov):
    """Yonelim acisini normalize piksel koordinatina donusturur. None = FOV disinda."""
    daz = _wrap_angle(azimuth_rad - cam_az)
    de = elevation_rad - cam_el
    half_h = h_fov / 2.0
    half_v = v_fov / 2.0
    if abs(daz) > half_h or abs(de) > half_v:
        return None
    px = 0.5 + daz / h_fov
    py = 0.5 - de / v_fov
    return px, py


def acoustic_snr(range_m, source_spl_db, background_db=40.0, wind_speed_ms=0.0):
    """Akustik SNR [dB]. Kuresel yayilim + ruzgar gurultusu."""
    if range_m < 1.0:
        range_m = 1.0
    spreading_loss = 20.0 * math.log10(range_m)
    wind_noise = 10.0 * math.log10(1.0 + wind_speed_ms ** 2)
    received = source_spl_db - spreading_loss
    noise_floor = background_db + wind_noise
    return received - noise_floor


def acoustic_doa_noise(snr_linear, baseline_m=1.0, frequency_hz=200.0):
    """DOA kestirimi standart sapmasi [rad]."""
    if snr_linear < 0.1:
        snr_linear = 0.1
    wavelength = SPEED_OF_SOUND / frequency_hz
    return wavelength / (baseline_m * math.sqrt(8.0 * snr_linear))


def generate_frequency_spectrum(classification, snr_db, num_bins=16):
    """Basitlestirilmis akustik frekans spektrumu [dB]."""
    dominant = DOMINANT_FREQ_HZ.get(classification, 200.0)
    spectrum = []
    noise_floor = -20.0
    for i in range(num_bins):
        freq = 50.0 + i * (8000.0 - 50.0) / num_bins
        dist_from_dominant = abs(freq - dominant)
        peak = max(snr_db - dist_from_dominant * 0.05, noise_floor)
        peak += random.gauss(0, 1.5)
        spectrum.append(peak)
    return spectrum


def _wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
