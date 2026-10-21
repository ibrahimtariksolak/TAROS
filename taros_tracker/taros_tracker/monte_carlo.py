"""Monte Carlo sensor fuzyon karsilastirmasi — ROS'tan bagimsiz."""

import math
import random

import numpy as np

from taros_sensor.sensor_models import (
    azimuth_elevation,
    db_to_linear,
    distance_3d,
    doppler_velocity,
    radar_angle_noise,
    radar_range_noise,
    radar_snr,
    radar_system_constant,
    swerling_i_pd,
)
from taros_tracker.ekf import EKF


K_SYS = radar_system_constant()
RCS_DBSM = -10.0
RADAR_POS = (0.0, 0.0, 15.0)
DT = 1.0
BEAMWIDTH_RAD = math.radians(3.0)
RANGE_RES = 30.0

EOIR_AZ_STD = 0.001
EOIR_EL_STD = 0.001
ACOUSTIC_AZ_STD = math.radians(5.0)
ACOUSTIC_EL_STD = math.radians(8.0)


def _generate_trajectory(n_steps, start, velocity):
    """Sabit hizli dogrusal yorunge uretir."""
    traj = []
    pos = list(start)
    for _ in range(n_steps):
        traj.append(tuple(pos))
        for i in range(3):
            pos[i] += velocity[i] * DT
    return traj


def _radar_measurement(true_pos, velocity):
    """Gurultulu radar olcumu uretir. None = tespit edilemedi."""
    rng = distance_3d(RADAR_POS, true_pos)
    snr_db = radar_snr(rng, RCS_DBSM, K_SYS)
    pd = swerling_i_pd(snr_db)
    if random.random() > pd:
        return None

    snr_lin = db_to_linear(snr_db)
    az_true, el_true = azimuth_elevation(RADAR_POS, true_pos)
    v_radial = doppler_velocity(velocity, RADAR_POS, true_pos)

    rng_noise = radar_range_noise(RANGE_RES, snr_lin)
    az_noise = radar_angle_noise(BEAMWIDTH_RAD, snr_lin)

    r_meas = rng + random.gauss(0, rng_noise)
    az_meas = az_true + random.gauss(0, az_noise)
    el_meas = el_true + random.gauss(0, az_noise)
    vr_meas = v_radial + random.gauss(0, 1.0)

    R = np.diag([
        rng_noise ** 2,
        az_noise ** 2,
        az_noise ** 2,
        1.0,
    ])
    return [r_meas, az_meas, el_meas, vr_meas], R


def _eoir_measurement(true_pos):
    """Gurultulu EO/IR yonelim olcumu uretir."""
    az, el = azimuth_elevation(RADAR_POS, true_pos)
    az_meas = az + random.gauss(0, EOIR_AZ_STD)
    el_meas = el + random.gauss(0, EOIR_EL_STD)
    R = np.diag([EOIR_AZ_STD ** 2, EOIR_EL_STD ** 2])
    return [az_meas, el_meas], R


def _acoustic_measurement(true_pos):
    """Gurultulu akustik DOA olcumu uretir."""
    az, el = azimuth_elevation(RADAR_POS, true_pos)
    az_meas = az + random.gauss(0, ACOUSTIC_AZ_STD)
    el_meas = el + random.gauss(0, ACOUSTIC_EL_STD)
    R = np.diag([ACOUSTIC_AZ_STD ** 2, ACOUSTIC_EL_STD ** 2])
    return [az_meas, el_meas], R


RADAR_SCAN_PERIOD = 4


def _run_single_trial(trajectory, velocity, sensors):
    """Tek bir Monte Carlo denemesini calistirir.

    Args:
        trajectory: (x,y,z) konumlari listesi.
        velocity: (vx,vy,vz) sabit hiz.
        sensors: Sensor isim kumesi {'radar','eoir','acoustic'}.

    Returns:
        (position_errors, velocity_errors, cov_traces) listesi.
    """
    init_pos = trajectory[0]
    init_state = [
        init_pos[0] + random.gauss(0, 200),
        init_pos[1] + random.gauss(0, 200),
        init_pos[2] + random.gauss(0, 50),
        velocity[0] + random.gauss(0, 10),
        velocity[1] + random.gauss(0, 10),
        velocity[2] + random.gauss(0, 5),
    ]
    init_cov = np.diag([
        500.0, 500.0, 200.0, 100.0, 100.0, 50.0])
    ekf = EKF(init_state, covariance=init_cov)

    pos_errors = []
    vel_errors = []
    cov_traces = []

    for step, true_pos in enumerate(trajectory[1:]):
        ekf.predict(DT)

        if 'radar' in sensors and step % RADAR_SCAN_PERIOD == 0:
            meas = _radar_measurement(true_pos, velocity)
            if meas is not None:
                z, R = meas
                ekf.update_radar(z, R)

        if 'eoir' in sensors:
            z, R = _eoir_measurement(true_pos)
            ekf.update_eoir(z, R)

        if 'acoustic' in sensors:
            z, R = _acoustic_measurement(true_pos)
            ekf.update_acoustic(z, R)

        est = ekf.position
        err = math.sqrt(sum(
            (est[i] - true_pos[i]) ** 2 for i in range(3)))
        pos_errors.append(err)

        v_est = ekf.velocity
        v_err = math.sqrt(sum(
            (v_est[i] - velocity[i]) ** 2 for i in range(3)))
        vel_errors.append(v_err)

        cov_traces.append(np.trace(ekf.P[:3, :3]))

    return pos_errors, vel_errors, cov_traces


def run_monte_carlo(
    n_runs=100,
    n_steps=60,
    start=(8000.0, 6000.0, 300.0),
    velocity=(-20.0, -15.0, -1.0),
    seed=42,
):
    """Monte Carlo fuzyon karsilastirmasini calistirir.

    Returns:
        Dict: her sensor konfigurasyonu icin sonuc istatistikleri.
    """
    random.seed(seed)
    np.random.seed(seed)

    trajectory = _generate_trajectory(n_steps, start, velocity)

    configs = {
        'radar_only': {'radar'},
        'eoir_only': {'eoir'},
        'acoustic_only': {'acoustic'},
        'radar_eoir': {'radar', 'eoir'},
        'radar_acoustic': {'radar', 'acoustic'},
        'all_sensors': {'radar', 'eoir', 'acoustic'},
    }

    results = {}

    for name, sensors in configs.items():
        all_pos = []
        all_vel = []
        all_cov = []

        for _ in range(n_runs):
            pe, ve, ct = _run_single_trial(
                trajectory, velocity, sensors)
            all_pos.append(pe)
            all_vel.append(ve)
            all_cov.append(ct)

        pos_arr = np.array(all_pos)
        vel_arr = np.array(all_vel)
        cov_arr = np.array(all_cov)

        results[name] = {
            'rmse_position': float(np.sqrt(
                np.mean(pos_arr[:, -20:] ** 2))),
            'rmse_velocity': float(np.sqrt(
                np.mean(vel_arr[:, -20:] ** 2))),
            'mean_cov_trace': float(np.mean(cov_arr[:, -20:])),
            'final_pos_error_mean': float(
                np.mean(pos_arr[:, -1])),
            'final_pos_error_std': float(
                np.std(pos_arr[:, -1])),
            'convergence_step': int(np.median([
                next(
                    (j for j in range(len(pe))
                     if pe[j] < 100.0),
                    len(pe))
                for pe in all_pos
            ])),
            'sensors': sorted(sensors),
        }

    return results


def format_report(results):
    """Sonuclari tablo biciminde dondurur."""
    lines = []
    lines.append(
        f'{"Konfigurasyon":<20} {"RMSE Pos[m]":>12} '
        f'{"RMSE Vel[m/s]":>14} {"Kov Izi":>10} '
        f'{"Yakinsama":>10}')
    lines.append('-' * 70)

    for name in [
        'radar_only', 'eoir_only', 'acoustic_only',
        'radar_eoir', 'radar_acoustic', 'all_sensors',
    ]:
        r = results[name]
        lines.append(
            f'{name:<20} {r["rmse_position"]:>12.2f} '
            f'{r["rmse_velocity"]:>14.2f} '
            f'{r["mean_cov_trace"]:>10.1f} '
            f'{r["convergence_step"]:>10d}')

    radar = results['radar_only']['rmse_position']
    fused = results['all_sensors']['rmse_position']
    if radar > 0:
        improv = (1.0 - fused / radar) * 100.0
        lines.append('')
        lines.append(
            f'Coklu sensor iyilestirmesi: '
            f'{improv:.1f}% (radar tek basina kiyasla)')

    return '\n'.join(lines)
