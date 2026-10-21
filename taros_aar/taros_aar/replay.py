"""AAR kayitlarindan replay ve metrik raporu olusturma."""

import json
import os

from taros_aar.metrics import generate_summary


def load_recording(filepath):
    """JSONL kayit dosyasini okur.

    Args:
        filepath: Kayit dosyasi yolu.

    Returns:
        Kayit dict listesi.
    """
    records = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def extract_engagement_records(records):
    """Kayitlardan angajman verilerini cikarir."""
    engagements = []
    for r in records:
        if r.get('topic') == 'engagement':
            engagements.append(r['data'])
    return engagements


def extract_track_snapshots(records):
    """Kayitlardan iz anlık goruntulerini cikarir."""
    snapshots = []
    for r in records:
        if r.get('topic') == 'tracks':
            snapshots.append({
                'timestamp': r['timestamp'],
                'count': r['data']['count'],
                'tracks': r['data']['tracks'],
            })
    return snapshots


def extract_weather_records(records):
    """Kayitlardan hava durumu verilerini cikarir."""
    weather = []
    for r in records:
        if r.get('topic') == 'weather':
            weather.append({
                'timestamp': r['timestamp'],
                **r['data'],
            })
    return weather


def compute_track_metrics(snapshots):
    """Iz anlik goruntulerinden metrikler hesaplar."""
    if not snapshots:
        return {
            'total_snapshots': 0,
            'max_concurrent_tracks': 0,
            'avg_tracks': 0.0,
            'max_threat_score': 0.0,
            'avg_confidence': 0.0,
        }

    counts = [s['count'] for s in snapshots]
    all_scores = []
    all_confs = []
    for s in snapshots:
        for t in s['tracks']:
            all_scores.append(t.get('threat_score', 0.0))
            all_confs.append(t.get('confidence', 0.0))

    return {
        'total_snapshots': len(snapshots),
        'max_concurrent_tracks': max(counts),
        'avg_tracks': sum(counts) / len(counts),
        'max_threat_score': max(all_scores) if all_scores else 0.0,
        'avg_confidence': (
            sum(all_confs) / len(all_confs)
            if all_confs else 0.0),
    }


def compute_timeline(snapshots, bin_seconds=10.0):
    """Zaman dilimlerine gore iz sayisi zaman serisi uretir.

    Args:
        snapshots: extract_track_snapshots ciktisi.
        bin_seconds: Zaman dilimi genisligi saniye cinsinden.

    Returns:
        (zaman_etiketleri, iz_sayilari) tuple.
    """
    if not snapshots:
        return [], []

    t0 = snapshots[0]['timestamp']
    bins = {}
    for s in snapshots:
        b = int((s['timestamp'] - t0) / bin_seconds)
        bins[b] = max(bins.get(b, 0), s['count'])

    max_bin = max(bins.keys())
    times = []
    counts = []
    for i in range(max_bin + 1):
        times.append(i * bin_seconds)
        counts.append(bins.get(i, 0))

    return times, counts


def generate_full_report(filepath):
    """Tam AAR raporu olusturur.

    Args:
        filepath: JSONL kayit dosyasi yolu.

    Returns:
        Rapor metni.
    """
    records = load_recording(filepath)

    engagements = extract_engagement_records(records)
    snapshots = extract_track_snapshots(records)
    weather = extract_weather_records(records)

    eng_summary = generate_summary(engagements)
    track_metrics = compute_track_metrics(snapshots)
    timeline_t, timeline_n = compute_timeline(snapshots)

    lines = []
    lines.append('=' * 60)
    lines.append('TAROS Muharebe Sonrasi Analiz Raporu')
    lines.append('=' * 60)
    lines.append('')

    lines.append(f'Kayit dosyasi: {os.path.basename(filepath)}')
    lines.append(f'Toplam kayit: {len(records)}')
    dur = (records[-1]['timestamp'] - records[0]['timestamp']
           if len(records) > 1 else 0.0)
    lines.append(f'Sure: {dur:.1f} saniye')
    lines.append('')

    lines.append('--- Angajman Metrikleri ---')
    lines.append(
        f'Toplam angajman: {eng_summary["total_engagements"]}')
    lines.append(
        f'Basari orani: {eng_summary["success_rate"]:.1%}')
    lines.append(
        f'Ort. sensor-atis suresi: '
        f'{eng_summary["avg_sensor_to_shoot"]:.2f} s')
    lines.append('')

    lines.append('--- Iz Takip Metrikleri ---')
    lines.append(
        f'Toplam anlik goruntu: '
        f'{track_metrics["total_snapshots"]}')
    lines.append(
        f'Maks. esanli hedef: '
        f'{track_metrics["max_concurrent_tracks"]}')
    lines.append(
        f'Ort. hedef sayisi: '
        f'{track_metrics["avg_tracks"]:.1f}')
    lines.append(
        f'Maks. tehdit skoru: '
        f'{track_metrics["max_threat_score"]:.1f}')
    lines.append(
        f'Ort. guvenilirlik: '
        f'{track_metrics["avg_confidence"]:.2f}')
    lines.append('')

    if weather:
        lines.append('--- Hava Kosullari ---')
        last_w = weather[-1]
        lines.append(f'Gorunurluk: {last_w["visibility"]:.0f} m')
        lines.append(
            f'Yagis: {last_w["rain_rate"]:.1f} mm/h')
        lines.append(f'Sicaklik: {last_w["temperature"]:.1f} C')
        lines.append('')

    if timeline_t:
        lines.append('--- Zaman Serisi (10s aralikli) ---')
        for t, n in zip(timeline_t, timeline_n):
            bar = '#' * n
            lines.append(f'  {t:6.0f}s | {n:3d} | {bar}')
        lines.append('')

    lines.append('=' * 60)
    return '\n'.join(lines)
