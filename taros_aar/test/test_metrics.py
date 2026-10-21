"""AAR metrik hesaplama birim testleri."""

from taros_aar.metrics import (
    average_detection_latency,
    engagement_success_rate,
    generate_summary,
    sensor_to_shoot_time,
    track_accuracy,
)


class TestEngagementSuccessRate:
    """Angazman basari orani testleri."""

    def test_all_success(self):
        """Tumu basarili ise oran 1.0 olmali."""
        records = [{'result': 1}, {'result': 1}]
        assert engagement_success_rate(records) == 1.0

    def test_all_miss(self):
        """Tumu iska ise oran 0.0 olmali."""
        records = [{'result': 2}, {'result': 2}]
        assert engagement_success_rate(records) == 0.0

    def test_mixed_results(self):
        """Karisik sonuclarda oran dogru hesaplanmali."""
        records = [{'result': 1}, {'result': 2}, {'result': 1}, {'result': 3}]
        assert engagement_success_rate(records) == 0.5

    def test_empty_records(self):
        """Bos liste icin oran 0.0 olmali."""
        assert engagement_success_rate([]) == 0.0


class TestDetectionLatency:
    """Tespit gecikmesi testleri."""

    def test_normal_latency(self):
        """Normal gecikme dogru hesaplanmali."""
        records = [
            {'detection_time': 10.0, 'first_track_time': 12.0},
            {'detection_time': 20.0, 'first_track_time': 23.0},
        ]
        avg = average_detection_latency(records)
        assert abs(avg - 2.5) < 0.01

    def test_empty_records(self):
        """Bos liste icin gecikme 0.0 olmali."""
        assert average_detection_latency([]) == 0.0

    def test_missing_fields(self):
        """Eksik alanlar atlanmali."""
        records = [{'detection_time': 10.0}]
        assert average_detection_latency(records) == 0.0


class TestTrackAccuracy:
    """Iz dogrulugu testleri."""

    def test_known_errors(self):
        """Bilinen hatalar dogru ortalanamali."""
        records = [
            {'position_error': 5.0},
            {'position_error': 15.0},
        ]
        assert abs(track_accuracy(records) - 10.0) < 0.01

    def test_empty_records(self):
        """Bos liste icin hata 0.0 olmali."""
        assert track_accuracy([]) == 0.0


class TestSensorToShootTime:
    """Sensorden atise gecen sure testleri."""

    def test_known_times(self):
        """Bilinen sureler dogru ortalanamali."""
        records = [
            {'sensor_to_shoot_time': 4.0},
            {'sensor_to_shoot_time': 6.0},
        ]
        assert abs(sensor_to_shoot_time(records) - 5.0) < 0.01

    def test_empty_records(self):
        """Bos liste icin sure 0.0 olmali."""
        assert sensor_to_shoot_time([]) == 0.0


class TestGenerateSummary:
    """Ozet rapor olusturma testleri."""

    def test_summary_keys(self):
        """Ozet beklenen anahtarlari icermeli."""
        records = [{'result': 1, 'sensor_to_shoot_time': 3.0}]
        summary = generate_summary(records)
        assert 'total_engagements' in summary
        assert 'success_rate' in summary
        assert summary['total_engagements'] == 1

    def test_summary_with_tracks(self):
        """Iz kayitlari verildiginde ek metrikler olmali."""
        eng = [{'result': 1, 'sensor_to_shoot_time': 3.0}]
        trk = [{'detection_time': 1.0,
                'first_track_time': 2.0,
                'position_error': 5.0}]
        summary = generate_summary(eng, trk)
        assert 'avg_detection_latency' in summary
        assert 'avg_track_accuracy' in summary
