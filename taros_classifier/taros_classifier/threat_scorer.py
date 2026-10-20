"""
Tehdit skorlama — coklu faktor agirlikli tehdit degerlendirmesi.

Her iz icin 0-100 arasi tehdit skoru ve renk seviyesi uretir.
"""

GREEN = 0
YELLOW = 1
ORANGE = 2
RED = 3

INTENT_WEIGHTS = {
    'ATTACK': 40.0,
    'RECONNAISSANCE': 20.0,
    'LOITERING': 15.0,
    'TRANSIT': 5.0,
    'UNKNOWN': 10.0,
}

CLASS_WEIGHTS = {
    'MILITARY': 15.0,
    'COMMERCIAL_QUAD': 5.0,
    'COMMERCIAL_FIXEDWING': 8.0,
    'UNKNOWN': 10.0,
}


def compute_threat_score(features, intent, classification):
    """
    Tehdit skoru hesaplar [0-100].

    Faktorler: menzil, varis suresi, hiz, niyet, sinif, irtifa.
    """
    range_to_base = features.get('range_to_base', 10000.0)
    closing_speed = features.get('closing_speed', 0.0)
    speed = features.get('speed', 0.0)
    altitude = features.get('altitude', 100.0)

    range_score = max(0.0, 20.0 * (1.0 - range_to_base / 10000.0))

    if closing_speed > 0 and range_to_base > 1.0:
        tta = range_to_base / closing_speed
        tta_score = max(0.0, 15.0 * (1.0 - tta / 300.0))
    else:
        tta_score = 0.0

    speed_score = min(10.0, speed / 5.0)

    intent_score = INTENT_WEIGHTS.get(intent, 10.0)
    class_score = CLASS_WEIGHTS.get(classification, 10.0)

    if altitude < 30.0:
        alt_score = 10.0
    elif altitude < 100.0:
        alt_score = 5.0
    else:
        alt_score = 0.0

    total = range_score + tta_score + speed_score + intent_score + class_score + alt_score
    return min(100.0, max(0.0, total))


def threat_level(score):
    """Skora gore tehdit seviyesi belirler."""
    if score >= 75.0:
        return RED
    if score >= 50.0:
        return ORANGE
    if score >= 25.0:
        return YELLOW
    return GREEN
