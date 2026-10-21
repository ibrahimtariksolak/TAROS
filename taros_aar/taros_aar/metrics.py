"""Muharebe sonrasi analiz metrik hesaplamalari."""


def engagement_success_rate(records):
    """Angazman basari oranini hesaplar.

    Args:
        records: Sonuc bilgisi iceren dict listesi.
            Her kayit 'result' anahtari tasir (1=SUCCESS).

    Returns:
        0.0-1.0 arasinda basari orani, kayit yoksa 0.0.
    """
    if not records:
        return 0.0
    success = sum(1 for r in records if r.get('result') == 1)
    return success / len(records)


def average_detection_latency(records):
    """Ortalama tespit gecikmesini hesaplar.

    Args:
        records: Her kayit 'detection_time' ve 'first_track_time'
            anahtarlari tasir (saniye cinsinden).

    Returns:
        Ortalama gecikme saniye cinsinden, kayit yoksa 0.0.
    """
    latencies = []
    for r in records:
        det = r.get('detection_time', 0.0)
        trk = r.get('first_track_time', 0.0)
        if det > 0 and trk > 0:
            latencies.append(trk - det)
    if not latencies:
        return 0.0
    return sum(latencies) / len(latencies)


def track_accuracy(records):
    """Iz konum dogrulugunun ortalama hatasini hesaplar.

    Args:
        records: Her kayit 'position_error' anahtari tasir
            (metre cinsinden).

    Returns:
        Ortalama konum hatasi metre cinsinden, kayit yoksa 0.0.
    """
    errors = [r['position_error'] for r in records
              if 'position_error' in r]
    if not errors:
        return 0.0
    return sum(errors) / len(errors)


def sensor_to_shoot_time(records):
    """Ortalama sensorden atise gecen sureyi hesaplar.

    Args:
        records: Her kayit 'sensor_to_shoot_time' anahtari tasir
            (saniye cinsinden).

    Returns:
        Ortalama sure saniye cinsinden, kayit yoksa 0.0.
    """
    times = [r['sensor_to_shoot_time'] for r in records
             if 'sensor_to_shoot_time' in r and r['sensor_to_shoot_time'] > 0]
    if not times:
        return 0.0
    return sum(times) / len(times)


def generate_summary(engagement_records, track_records=None):
    """Tum metrikleri ozetleyen bir dict olusturur.

    Args:
        engagement_records: Angazman kayitlari.
        track_records: Iz kayitlari (opsiyonel).

    Returns:
        Metrik anahtarlari ve degerlerini iceren dict.
    """
    summary = {
        'total_engagements': len(engagement_records),
        'success_rate': engagement_success_rate(engagement_records),
        'avg_sensor_to_shoot': sensor_to_shoot_time(engagement_records),
    }

    if track_records:
        summary['avg_detection_latency'] = average_detection_latency(
            track_records)
        summary['avg_track_accuracy'] = track_accuracy(track_records)

    return summary
