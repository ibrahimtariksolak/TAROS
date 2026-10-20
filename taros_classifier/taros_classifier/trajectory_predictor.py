"""
Yorunge kestirimi — dogrusal ve sabit donus modeli.

Mevcut durum vektorunden gelecek konumu tahmin eder. En yakin yaklasma
mesafesi ve tahmini carpisma noktasi hesaplar.
"""

import math


def predict_linear(position, velocity, dt):
    """Dogrusal yorunge tahmini."""
    return (
        position[0] + velocity[0] * dt,
        position[1] + velocity[1] * dt,
        position[2] + velocity[2] * dt,
    )


def predict_trajectory(position, velocity, horizon_s=30.0, step_s=1.0):
    """Zaman ufku boyunca konum dizisi uretir."""
    points = []
    t = 0.0
    while t <= horizon_s:
        points.append(predict_linear(position, velocity, t))
        t += step_s
    return points


def closest_approach(position, velocity, target=(0.0, 0.0, 0.0), horizon_s=60.0):
    """
    Hedefe en yakin yaklasma mesafesini ve zamanini hesaplar.

    Analitik cozum: d(t) = |P + V*t - T| minimize edilir.
    """
    dx = position[0] - target[0]
    dy = position[1] - target[1]
    dz = position[2] - target[2]
    vx, vy, vz = velocity

    a = vx * vx + vy * vy + vz * vz
    b = 2.0 * (dx * vx + dy * vy + dz * vz)

    if a < 1e-10:
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist, 0.0

    t_min = -b / (2.0 * a)
    t_min = max(0.0, min(t_min, horizon_s))

    px = dx + vx * t_min
    py = dy + vy * t_min
    pz = dz + vz * t_min
    dist = math.sqrt(px * px + py * py + pz * pz)

    return dist, t_min


def predicted_impact(position, velocity, target=(0.0, 0.0, 0.0)):
    """En yakin yaklasma noktasini dondurur."""
    _, t = closest_approach(position, velocity, target)
    return predict_linear(position, velocity, t)
