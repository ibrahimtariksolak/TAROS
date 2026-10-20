"""
TDOA trilateration ile opertor konumlandirma.

Birden fazla sensor istasyonunun varis zamani farki (TDOA) olcumlerinden
RF vericisinin (drone operatoru) konumunu tahmin eder.
ROS'tan bagimsiz matematik modulu.
"""

import math

import numpy as np


def tdoa_localize(sensor_positions, tdoa_measurements):
    """
    TDOA olcumlerinden verici konumu tahmin eder.

    sensor_positions: [(x1,y1,z1), (x2,y2,z2), ...] en az 4 sensor
    tdoa_measurements: [dt12, dt13, dt14, ...] referans sensore gore farklar [s]

    Donus: (x, y, z) tahmini konum veya None
    """
    n = len(sensor_positions)
    if n < 4 or len(tdoa_measurements) < n - 1:
        return None

    c = 2.998e8

    ref = np.array(sensor_positions[0], dtype=np.float64)
    A = []
    b = []

    for i in range(1, n):
        si = np.array(sensor_positions[i], dtype=np.float64)
        d_diff = tdoa_measurements[i - 1] * c

        row = 2.0 * (si - ref)
        rhs = (
            np.dot(si, si) - np.dot(ref, ref)
            - d_diff ** 2
        )
        A.append(row)
        b.append(rhs)

    A = np.array(A)
    b = np.array(b)

    try:
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return tuple(result)
    except np.linalg.LinAlgError:
        return None


def compute_cep(estimated, true_position):
    """Dairesel hata olasiligi (2D yatay mesafe) [m]."""
    dx = estimated[0] - true_position[0]
    dy = estimated[1] - true_position[1]
    return math.sqrt(dx * dx + dy * dy)
