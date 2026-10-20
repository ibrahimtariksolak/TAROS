"""Siniflandirma, niyet analizi, tehdit skorlama ve yorunge kestirimi testleri."""

import math

from taros_classifier.intent_analyzer import (
    INTENT_ATTACK,
    INTENT_LOITERING,
    INTENT_RECONNAISSANCE,
    INTENT_TRANSIT,
    INTENT_UNKNOWN,
    classify_intent,
)
from taros_classifier.threat_scorer import (
    GREEN,
    ORANGE,
    RED,
    YELLOW,
    compute_threat_score,
    threat_level,
)
from taros_classifier.trajectory_predictor import (
    closest_approach,
    predict_linear,
    predict_trajectory,
    predicted_impact,
)


class TestIntentAnalysis:
    """Niyet siniflandirma kurallari."""

    def test_attack_intent(self):
        features = {
            'speed': 30.0,
            'heading_rate': 0.0,
            'range_to_base': 1500.0,
            'closing_speed': 25.0,
            'altitude': 50.0,
            'altitude_rate': -5.0,
        }
        assert classify_intent(features) == INTENT_ATTACK

    def test_loitering_intent(self):
        features = {
            'speed': 8.0,
            'heading_rate': math.radians(20.0),
            'range_to_base': 800.0,
            'closing_speed': 0.0,
            'altitude': 100.0,
            'altitude_rate': 0.0,
        }
        assert classify_intent(features) == INTENT_LOITERING

    def test_recon_intent(self):
        features = {
            'speed': 20.0,
            'heading_rate': math.radians(8.0),
            'range_to_base': 3000.0,
            'closing_speed': 2.0,
            'altitude': 200.0,
            'altitude_rate': 0.0,
        }
        assert classify_intent(features) == INTENT_RECONNAISSANCE

    def test_transit_intent(self):
        features = {
            'speed': 25.0,
            'heading_rate': math.radians(1.0),
            'range_to_base': 8000.0,
            'closing_speed': 0.0,
            'altitude': 300.0,
            'altitude_rate': 0.0,
        }
        assert classify_intent(features) == INTENT_TRANSIT

    def test_unknown_intent_stationary(self):
        features = {
            'speed': 0.0,
            'heading_rate': 0.0,
            'range_to_base': 5000.0,
            'closing_speed': 0.0,
            'altitude': 100.0,
            'altitude_rate': 0.0,
        }
        assert classify_intent(features) == INTENT_UNKNOWN


class TestThreatScorer:
    """Tehdit skorlama."""

    def test_attack_high_score(self):
        features = {
            'range_to_base': 500.0,
            'closing_speed': 30.0,
            'speed': 30.0,
            'altitude': 20.0,
        }
        score = compute_threat_score(features, 'ATTACK', 'MILITARY')
        assert score >= 75.0

    def test_transit_low_score(self):
        features = {
            'range_to_base': 9000.0,
            'closing_speed': 0.0,
            'speed': 15.0,
            'altitude': 300.0,
        }
        score = compute_threat_score(features, 'TRANSIT', 'COMMERCIAL_QUAD')
        assert score < 25.0

    def test_score_range(self):
        features = {
            'range_to_base': 5000.0,
            'closing_speed': 10.0,
            'speed': 20.0,
            'altitude': 100.0,
        }
        score = compute_threat_score(features, 'RECONNAISSANCE', 'UNKNOWN')
        assert 0.0 <= score <= 100.0

    def test_closer_higher_score(self):
        features_far = {
            'range_to_base': 8000.0, 'closing_speed': 10.0,
            'speed': 20.0, 'altitude': 100.0,
        }
        features_near = {
            'range_to_base': 1000.0, 'closing_speed': 10.0,
            'speed': 20.0, 'altitude': 100.0,
        }
        score_far = compute_threat_score(features_far, 'TRANSIT', 'UNKNOWN')
        score_near = compute_threat_score(features_near, 'TRANSIT', 'UNKNOWN')
        assert score_near > score_far

    def test_threat_level_green(self):
        assert threat_level(10.0) == GREEN

    def test_threat_level_yellow(self):
        assert threat_level(30.0) == YELLOW

    def test_threat_level_orange(self):
        assert threat_level(60.0) == ORANGE

    def test_threat_level_red(self):
        assert threat_level(85.0) == RED

    def test_threat_level_boundaries(self):
        assert threat_level(0.0) == GREEN
        assert threat_level(25.0) == YELLOW
        assert threat_level(50.0) == ORANGE
        assert threat_level(75.0) == RED
        assert threat_level(100.0) == RED


class TestTrajectoryPredictor:
    """Yorunge kestirimi."""

    def test_linear_prediction(self):
        pos = predict_linear((0, 0, 100), (10, 0, 0), 5.0)
        assert abs(pos[0] - 50.0) < 1e-6
        assert abs(pos[1]) < 1e-6
        assert abs(pos[2] - 100.0) < 1e-6

    def test_trajectory_length(self):
        points = predict_trajectory((0, 0, 0), (10, 0, 0), horizon_s=10.0, step_s=1.0)
        assert len(points) == 11

    def test_closest_approach_stationary(self):
        dist, t = closest_approach((100, 0, 0), (0, 0, 0))
        assert abs(dist - 100.0) < 1e-6
        assert t == 0.0

    def test_closest_approach_head_on(self):
        dist, t = closest_approach((1000, 0, 0), (-20, 0, 0))
        assert dist < 1.0
        assert abs(t - 50.0) < 1.0

    def test_closest_approach_passing(self):
        dist, _ = closest_approach((1000, 100, 0), (-20, 0, 0))
        assert abs(dist - 100.0) < 5.0

    def test_closest_approach_receding(self):
        dist, t = closest_approach((1000, 0, 0), (20, 0, 0))
        assert dist >= 1000.0
        assert t == 0.0

    def test_predicted_impact_approaching(self):
        impact = predicted_impact((1000, 0, 0), (-20, 0, 0))
        assert abs(impact[0]) < 1.0

    def test_zero_velocity(self):
        dist, t = closest_approach((500, 0, 0), (0, 0, 0))
        assert math.isfinite(dist)
        assert t == 0.0
