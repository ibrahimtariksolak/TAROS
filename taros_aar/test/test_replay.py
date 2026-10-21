"""AAR replay ve rapor olusturma birim testleri."""

import json
import os
import tempfile

from taros_aar.replay import (
    compute_timeline,
    compute_track_metrics,
    extract_engagement_records,
    extract_track_snapshots,
    extract_weather_records,
    generate_full_report,
    load_recording,
)


def _make_recording(entries):
    """Gecici JSONL dosyasi olusturur."""
    fd, path = tempfile.mkstemp(suffix='.jsonl')
    with os.fdopen(fd, 'w') as f:
        for entry in entries:
            f.write(json.dumps(entry) + '\n')
    return path


SAMPLE_RECORDS = [
    {
        'timestamp': 100.0,
        'topic': 'tracks',
        'data': {
            'count': 2,
            'tracks': [
                {
                    'track_id': 1, 'status': 1,
                    'classification': 'COMMERCIAL_QUAD',
                    'position': [1000, 500, 200],
                    'speed': 15.0, 'threat_score': 65.0,
                    'confidence': 0.85,
                },
                {
                    'track_id': 2, 'status': 0,
                    'classification': 'UNKNOWN',
                    'position': [2000, 1000, 150],
                    'speed': 10.0, 'threat_score': 30.0,
                    'confidence': 0.4,
                },
            ],
        },
    },
    {
        'timestamp': 110.0,
        'topic': 'tracks',
        'data': {
            'count': 3,
            'tracks': [
                {
                    'track_id': 1, 'status': 1,
                    'classification': 'COMMERCIAL_QUAD',
                    'position': [900, 480, 195],
                    'speed': 14.0, 'threat_score': 70.0,
                    'confidence': 0.9,
                },
                {
                    'track_id': 2, 'status': 1,
                    'classification': 'COMMERCIAL_QUAD',
                    'position': [1900, 980, 145],
                    'speed': 11.0, 'threat_score': 45.0,
                    'confidence': 0.6,
                },
                {
                    'track_id': 3, 'status': 0,
                    'classification': 'UNKNOWN',
                    'position': [3000, 1500, 300],
                    'speed': 20.0, 'threat_score': 20.0,
                    'confidence': 0.3,
                },
            ],
        },
    },
    {
        'timestamp': 115.0,
        'topic': 'engagement',
        'data': {
            'engagement_id': 1,
            'track_id': 1,
            'unit_id': 'alpha',
            'effector_type': 0,
            'result': 1,
            'sensor_to_shoot_time': 4.5,
            'miss_distance': 0.0,
        },
    },
    {
        'timestamp': 120.0,
        'topic': 'weather',
        'data': {
            'visibility': 8000.0,
            'rain_rate': 2.0,
            'fog_density': 0.0,
            'wind': [3.0, 1.0, 0.0],
            'is_night': False,
            'temperature': 22.0,
        },
    },
    {
        'timestamp': 125.0,
        'topic': 'engagement',
        'data': {
            'engagement_id': 2,
            'track_id': 2,
            'unit_id': 'alpha',
            'effector_type': 0,
            'result': 2,
            'sensor_to_shoot_time': 6.0,
            'miss_distance': 15.0,
        },
    },
]


class TestLoadRecording:
    """Kayit yuklemeleri testleri."""

    def test_load_valid_file(self):
        """Gecerli JSONL dosyasi yuklenmeli."""
        path = _make_recording(SAMPLE_RECORDS)
        try:
            records = load_recording(path)
            assert len(records) == 5
        finally:
            os.unlink(path)

    def test_load_empty_file(self):
        """Bos dosya bos liste dondurmeli."""
        path = _make_recording([])
        try:
            records = load_recording(path)
            assert records == []
        finally:
            os.unlink(path)


class TestExtractors:
    """Veri cikarici fonksiyon testleri."""

    def test_extract_engagements(self):
        """Angajman kayitlari dogru cikarilmali."""
        engagements = extract_engagement_records(SAMPLE_RECORDS)
        assert len(engagements) == 2
        assert engagements[0]['result'] == 1
        assert engagements[1]['result'] == 2

    def test_extract_track_snapshots(self):
        """Iz anlık goruntuleri dogru cikarilmali."""
        snapshots = extract_track_snapshots(SAMPLE_RECORDS)
        assert len(snapshots) == 2
        assert snapshots[0]['count'] == 2
        assert snapshots[1]['count'] == 3

    def test_extract_weather(self):
        """Hava durumu kayitlari dogru cikarilmali."""
        weather = extract_weather_records(SAMPLE_RECORDS)
        assert len(weather) == 1
        assert weather[0]['visibility'] == 8000.0


class TestTrackMetrics:
    """Iz metrik testleri."""

    def test_known_snapshots(self):
        """Bilinen anlık goruntulerden metrikler dogru olmali."""
        snapshots = extract_track_snapshots(SAMPLE_RECORDS)
        m = compute_track_metrics(snapshots)
        assert m['max_concurrent_tracks'] == 3
        assert m['total_snapshots'] == 2
        assert m['max_threat_score'] == 70.0

    def test_empty_snapshots(self):
        """Bos liste icin varsayilan degerler donmeli."""
        m = compute_track_metrics([])
        assert m['total_snapshots'] == 0
        assert m['max_concurrent_tracks'] == 0


class TestTimeline:
    """Zaman serisi testleri."""

    def test_timeline_bins(self):
        """Zaman dilimleri dogru olusturulmali."""
        snapshots = extract_track_snapshots(SAMPLE_RECORDS)
        times, counts = compute_timeline(snapshots, bin_seconds=5.0)
        assert len(times) == len(counts)
        assert len(times) > 0

    def test_empty_timeline(self):
        """Bos liste bos zaman serisi dondurmeli."""
        times, counts = compute_timeline([])
        assert times == []
        assert counts == []


class TestFullReport:
    """Tam rapor olusturma testleri."""

    def test_report_contains_sections(self):
        """Rapor tum bolumleri icermeli."""
        path = _make_recording(SAMPLE_RECORDS)
        try:
            report = generate_full_report(path)
            assert 'Angajman' in report
            assert 'Iz Takip' in report
            assert 'Hava' in report
            assert 'Zaman Serisi' in report
        finally:
            os.unlink(path)

    def test_report_engagement_stats(self):
        """Rapor angajman istatistiklerini icermeli."""
        path = _make_recording(SAMPLE_RECORDS)
        try:
            report = generate_full_report(path)
            assert '50.0%' in report
        finally:
            os.unlink(path)
