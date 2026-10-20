"""
Niyet analizi — iz davranisina gore hedefin amacini belirler.

Ozellik vektorleri uzerinden kural tabanli siniflandirma: yon degisim hizi,
hiz profili, hedefe yaklasilma durumu, irtifa degisimi.
"""

import math

INTENT_TRANSIT = 'TRANSIT'
INTENT_LOITERING = 'LOITERING'
INTENT_RECONNAISSANCE = 'RECONNAISSANCE'
INTENT_ATTACK = 'ATTACK'
INTENT_UNKNOWN = 'UNKNOWN'


def classify_intent(features):
    """
    Iz ozelliklerine gore niyet belirler.

    features: dict with keys:
        speed, heading_rate, range_to_base, closing_speed,
        altitude, altitude_rate, time_in_zone
    """
    speed = features.get('speed', 0.0)
    heading_rate = abs(features.get('heading_rate', 0.0))
    range_to_base = features.get('range_to_base', 10000.0)
    closing_speed = features.get('closing_speed', 0.0)
    altitude = features.get('altitude', 100.0)
    altitude_rate = features.get('altitude_rate', 0.0)

    if closing_speed > 5.0 and range_to_base < 3000.0 and altitude_rate <= 0:
        return INTENT_ATTACK

    if heading_rate > math.radians(15.0) and speed < 15.0 and range_to_base < 2000.0:
        return INTENT_LOITERING

    if heading_rate > math.radians(5.0) and range_to_base < 5000.0 and altitude > 50.0:
        return INTENT_RECONNAISSANCE

    if speed > 5.0 and heading_rate < math.radians(3.0):
        return INTENT_TRANSIT

    return INTENT_UNKNOWN


def compute_features(track, prev_heading=None, prev_altitude=None, dt=0.1):
    """
    Track mesajindan ozellik vektoru cikarir.

    prev_heading ve prev_altitude onceki degerlerdir; None ise oran
    hesaplanmaz.
    """
    speed = track.speed
    heading = track.heading
    altitude = track.altitude
    range_to_base = track.range_to_base

    heading_rate = 0.0
    if prev_heading is not None and dt > 0:
        dh = heading - prev_heading
        while dh > math.pi:
            dh -= 2.0 * math.pi
        while dh < -math.pi:
            dh += 2.0 * math.pi
        heading_rate = dh / dt

    altitude_rate = 0.0
    if prev_altitude is not None and dt > 0:
        altitude_rate = (altitude - prev_altitude) / dt

    vx = track.velocity.x
    vy = track.velocity.y
    px = track.position.x
    py = track.position.y
    dist_horiz = math.hypot(px, py)
    closing_speed = 0.0
    if dist_horiz > 1e-3:
        closing_speed = -(px * vx + py * vy) / dist_horiz

    return {
        'speed': speed,
        'heading_rate': heading_rate,
        'range_to_base': range_to_base,
        'closing_speed': closing_speed,
        'altitude': altitude,
        'altitude_rate': altitude_rate,
    }
