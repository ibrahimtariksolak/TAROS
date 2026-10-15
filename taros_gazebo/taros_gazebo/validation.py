"""
Hedef uretim isteklerinin dogrulanmasi.

Bu modul bilincli olarak ROS'tan bagimsizdir: dogrulama kurallari
simulasyon ayakta olmadan test edilebilsin diye saf Python tutuldu.
"""

import math

DRONE_TYPES = ('quadrotor', 'fixedwing', 'hexarotor')

BEHAVIORS = ('TRANSIT', 'LOITER', 'ATTACK', 'RECON')

# Hedefler ses altindan hizli olmaz, dunya da kilometre olceginde;
# bu sinirlar sacma degerleri kapida keser.
MAX_TARGET_SPEED = 300.0
MAX_COORDINATE = 50_000.0
MAX_WAYPOINTS = 64


def _iter_axes(point):
    for axis in ('x', 'y', 'z'):
        yield axis, getattr(point, axis)


def validate_spawn_request(drone_type, speed, spawn_position, waypoints, behavior=None):
    """
    Hedef uretim istegini dogrular.

    Sorun varsa insan tarafindan okunabilir hata metni, aksi halde None
    dondurur.

    Iki sinif riski kapatir:

    1. ``drone_type`` dosya yoluna giriyor. Whitelist disi bir deger
       ``'../../etc/passwd'`` tarzi yol kacisina imkan verirdi.
    2. Sayisal alanlar NaN/sonsuz olabilir. Bunlar sessizce Gazebo'ya ve
       ground truth akisina sizar, oradan da takip/fuzyon zincirini geri
       donulemez sekilde zehirler.
    """
    if drone_type not in DRONE_TYPES:
        return f'Bilinmeyen tip: {drone_type!r}. Gecerli: {list(DRONE_TYPES)}'

    if behavior is not None and behavior not in BEHAVIORS:
        return f'Bilinmeyen davranis: {behavior!r}. Gecerli: {list(BEHAVIORS)}'

    if not math.isfinite(speed):
        return f'Hiz sonlu bir sayi olmali, alinan: {speed}'
    if not 0.0 < speed <= MAX_TARGET_SPEED:
        return f'Hiz (0, {MAX_TARGET_SPEED}] araliginda olmali, alinan: {speed}'

    if len(waypoints) > MAX_WAYPOINTS:
        return f'Rota noktasi sayisi {MAX_WAYPOINTS} siniri asti: {len(waypoints)}'

    labelled = [('spawn_position', spawn_position)]
    labelled += [(f'waypoint[{i}]', wp) for i, wp in enumerate(waypoints)]

    for name, point in labelled:
        for axis, value in _iter_axes(point):
            if not math.isfinite(value):
                return f'{name}.{axis} sonlu degil: {value}'
            if abs(value) > MAX_COORDINATE:
                return f'{name}.{axis} dunya sinirini asti: {value}'

    return None
