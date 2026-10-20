"""
Veri iliskilendirme: Mahalanobis kapilama + Macar algoritmasi.

Coklu hedef takibinde her olcumun hangi ize ait oldugunu belirler.
Eslesmeyen olcumler yeni iz adayidir; eslesmeyen izler coasting'e girer.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment


def gated_cost_matrix(tracks, measurements, sensor_type, gate_threshold=16.0):
    """
    Iz-olcum maliyet matrisi olusturur (Mahalanobis mesafesi).

    tracks: EKF listesi
    measurements: olcum listesi (her biri numpy array)
    sensor_type: 'radar' / 'eoir' / 'acoustic'
    gate_threshold: chi-kare esik degeri

    Dondurur: (cost_matrix, gated_mask)
    - cost_matrix: (n_tracks, n_meas) Mahalanobis mesafe matrisi
    - gated_mask: boolean matrisi, True = esik icinde
    """
    n_tracks = len(tracks)
    n_meas = len(measurements)

    if n_tracks == 0 or n_meas == 0:
        return np.empty((n_tracks, n_meas)), np.zeros((n_tracks, n_meas), dtype=bool)

    cost = np.full((n_tracks, n_meas), 1e6)
    gated = np.zeros((n_tracks, n_meas), dtype=bool)

    for i, track in enumerate(tracks):
        for j, meas in enumerate(measurements):
            d = track.mahalanobis(meas, sensor_type)
            cost[i, j] = d
            if d < gate_threshold:
                gated[i, j] = True

    return cost, gated


def associate(tracks, measurements, sensor_type, gate_threshold=16.0):
    """
    Macar algoritmasi ile iz-olcum eslestirir.

    Dondurur: (matches, unmatched_tracks, unmatched_measurements)
    - matches: [(track_idx, meas_idx), ...]
    - unmatched_tracks: [track_idx, ...]
    - unmatched_measurements: [meas_idx, ...]
    """
    n_tracks = len(tracks)
    n_meas = len(measurements)

    if n_tracks == 0:
        return [], [], list(range(n_meas))
    if n_meas == 0:
        return [], list(range(n_tracks)), []

    cost, gated = gated_cost_matrix(tracks, measurements, sensor_type, gate_threshold)

    assignment_cost = cost.copy()
    assignment_cost[~gated] = 1e6

    row_ind, col_ind = linear_sum_assignment(assignment_cost)

    matches = []
    matched_tracks = set()
    matched_meas = set()

    for r, c in zip(row_ind, col_ind):
        if gated[r, c]:
            matches.append((r, c))
            matched_tracks.add(r)
            matched_meas.add(c)

    unmatched_tracks = [i for i in range(n_tracks) if i not in matched_tracks]
    unmatched_meas = [j for j in range(n_meas) if j not in matched_meas]

    return matches, unmatched_tracks, unmatched_meas
