"""
Hedef uretim istegi dogrulamasinin regresyon testleri.

Bu testler ROS veya Gazebo gerektirmez; taros_gazebo.validation modulu
bilincli olarak saf Python tutuldu.
"""

import pytest

from taros_gazebo.validation import (
    MAX_COORDINATE,
    MAX_TARGET_SPEED,
    MAX_WAYPOINTS,
    validate_spawn_request,
)

INF = float('inf')
NAN = float('nan')


class Point:
    """geometry_msgs/Point yerine gecen hafif test nesnesi."""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


def _request(drone_type='quadrotor', speed=15.0, spawn=None, waypoints=None):
    return {
        'drone_type': drone_type,
        'speed': speed,
        'spawn_position': spawn if spawn is not None else Point(0, 0, 50),
        'waypoints': waypoints if waypoints is not None else [Point(0, 0, 30)],
    }


def test_gecerli_istek_kabul_edilir():
    assert validate_spawn_request(**_request()) is None


@pytest.mark.parametrize('drone_type', ['quadrotor', 'fixedwing', 'hexarotor'])
def test_tum_whitelist_tipleri_kabul_edilir(drone_type):
    assert validate_spawn_request(**_request(drone_type=drone_type)) is None


@pytest.mark.parametrize('hostile', [
    '../../../etc/passwd',
    '..',
    '/etc/shadow',
    'quadrotor/../../../root',
    'quadrotor\x00',
    '',
])
def test_yol_kacisi_reddedilir(hostile):
    """drone_type dosya yoluna girdigi icin whitelist disi deger reddedilmeli."""
    assert validate_spawn_request(**_request(drone_type=hostile)) is not None


def test_bilinmeyen_tip_reddedilir():
    assert validate_spawn_request(**_request(drone_type='f35')) is not None


@pytest.mark.parametrize('speed', [NAN, INF, -INF, -10.0, 0.0, MAX_TARGET_SPEED + 1])
def test_gecersiz_hiz_reddedilir(speed):
    assert validate_spawn_request(**_request(speed=speed)) is not None


def test_sinirdaki_hiz_kabul_edilir():
    assert validate_spawn_request(**_request(speed=MAX_TARGET_SPEED)) is None


@pytest.mark.parametrize('bad', [
    Point(NAN, 0, 50),
    Point(0, INF, 50),
    Point(0, 0, NAN),
    Point(MAX_COORDINATE + 1, 0, 50),
])
def test_gecersiz_spawn_koordinati_reddedilir(bad):
    assert validate_spawn_request(**_request(spawn=bad)) is not None


@pytest.mark.parametrize('bad', [
    Point(INF, 0, 30),
    Point(0, NAN, 30),
    Point(0, 0, -MAX_COORDINATE - 1),
])
def test_gecersiz_waypoint_reddedilir(bad):
    assert validate_spawn_request(**_request(waypoints=[bad])) is not None


def test_waypoint_sayisi_siniri():
    tam_sinirda = [Point(i, 0, 30) for i in range(MAX_WAYPOINTS)]
    assert validate_spawn_request(**_request(waypoints=tam_sinirda)) is None

    bir_fazla = [Point(i, 0, 30) for i in range(MAX_WAYPOINTS + 1)]
    assert validate_spawn_request(**_request(waypoints=bir_fazla)) is not None


def test_bos_waypoint_listesi_kabul_edilir():
    """Rota noktasi olmayan hedef (ornegin LOITER) gecerlidir."""
    assert validate_spawn_request(**_request(waypoints=[])) is None


def test_bilinmeyen_davranis_reddedilir():
    assert validate_spawn_request(**_request(), behavior='KAMIKAZE') is not None


@pytest.mark.parametrize('behavior', ['TRANSIT', 'LOITER', 'ATTACK', 'RECON'])
def test_gecerli_davranislar_kabul_edilir(behavior):
    assert validate_spawn_request(**_request(), behavior=behavior) is None


def test_hata_mesaji_okunabilir():
    """Hata metni reddedilen degeri icermeli, yoksa hata ayiklamak zor."""
    error = validate_spawn_request(**_request(drone_type='f35'))
    assert 'f35' in error
    assert 'quadrotor' in error
