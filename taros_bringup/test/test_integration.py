"""Uctan uca entegrasyon testi — tum modullerin birlikte calismasi."""

import json
import math
import os
import random
import tempfile

import numpy as np

from taros_aar.replay import (
    compute_track_metrics,
    extract_engagement_records,
    extract_track_snapshots,
    generate_full_report,
    load_recording,
)
from taros_classifier.intent_analyzer import classify_intent
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
    predict_trajectory,
)
from taros_iads.wta_optimizer import DefenseUnit, WTAOptimizer
from taros_navigation.path_planner import GridMap, plan_path
from taros_sensor.sensor_models import (
    azimuth_elevation,
    distance_3d,
    doppler_velocity,
    radar_snr,
    radar_system_constant,
    swerling_i_pd,
)
from taros_terrain.terrain_models import (
    ElevationGrid,
    check_line_of_sight,
)
from taros_tracker.ekf import EKF
from taros_tracker.monte_carlo import run_monte_carlo


class TestEndToEndPipeline:
    """Algilama-takip-siniflandirma-karar zinciri testi."""

    def test_full_chain(self):
        """Tam zincir: sensor -> EKF -> siniflandirma -> WTA."""
        random.seed(42)
        np.random.seed(42)

        k_sys = radar_system_constant()
        radar_pos = (0.0, 0.0, 15.0)
        target_pos = (2000.0, 1000.0, 200.0)
        target_vel = (-15.0, -5.0, -1.0)

        rng = distance_3d(radar_pos, target_pos)
        snr_db = radar_snr(rng, -10.0, k_sys)
        pd = swerling_i_pd(snr_db)
        assert pd > 0.3

        az, el = azimuth_elevation(radar_pos, target_pos)
        vr = doppler_velocity(target_vel, radar_pos, target_pos)

        r_meas = rng + random.gauss(0, 15.0)
        az_meas = az + random.gauss(0, 0.01)
        el_meas = el + random.gauss(0, 0.01)
        vr_meas = vr + random.gauss(0, 1.0)

        ekf = EKF([
            target_pos[0] + random.gauss(0, 50),
            target_pos[1] + random.gauss(0, 50),
            target_pos[2] + random.gauss(0, 20),
            target_vel[0] + random.gauss(0, 5),
            target_vel[1] + random.gauss(0, 5),
            target_vel[2] + random.gauss(0, 2),
        ])

        for _ in range(20):
            ekf.predict(1.0)
            ekf.update_radar(
                [r_meas, az_meas, el_meas, vr_meas])
            ekf.update_eoir([az_meas, el_meas])

        est = ekf.position
        err = distance_3d(est, target_pos)
        assert err < 200.0

        speed = math.sqrt(sum(v ** 2 for v in target_vel))
        features = {
            'speed': speed,
            'heading_rate': 0.0,
            'range_to_base': rng,
            'closing_speed': abs(vr),
            'altitude': target_pos[2],
            'altitude_rate': target_vel[2],
        }
        intent = classify_intent(features)
        assert intent in (
            'ATTACK', 'LOITERING', 'RECONNAISSANCE',
            'TRANSIT', 'UNKNOWN')

        score = compute_threat_score(
            features, intent, 'COMMERCIAL_QUAD')
        assert 0 <= score <= 100
        level = threat_level(score)
        assert level in (GREEN, YELLOW, ORANGE, RED)

        traj = predict_trajectory(
            list(target_pos), list(target_vel),
            horizon_s=10.0, step_s=1.0)
        assert len(traj) >= 10

        cpa_dist, cpa_time = closest_approach(
            list(target_pos), list(target_vel),
            [0.0, 0.0, 0.0])
        assert cpa_dist >= 0.0
        assert cpa_time >= 0.0

        units = [
            DefenseUnit('alpha', 0.0, 0.0, 5000.0),
            DefenseUnit('bravo', 3000.0, 0.0, 4000.0),
        ]
        optimizer = WTAOptimizer(units)
        threats = [(1, target_pos[0], target_pos[1], 1)]
        assignments = optimizer.assign(threats)
        assert len(assignments) >= 1

    def test_terrain_affects_los(self):
        """Arazi LOS ile sensor tespiti etkilenir."""
        grid = ElevationGrid(5000.0, 5000.0, 10.0)
        grid.add_obstacle(2500.0, 2500.0, 200.0, 100.0)

        radar = (500.0, 2500.0, 15.0)
        target_clear = (4500.0, 500.0, 200.0)
        target_blocked = (4500.0, 2500.0, 10.0)

        assert check_line_of_sight(
            grid, radar, target_clear) is True
        assert check_line_of_sight(
            grid, radar, target_blocked) is False

    def test_navigation_to_new_position(self):
        """UGV engelden kacinarak yeni mevziye ulasmali."""
        gm = GridMap(500.0, 500.0, 5.0)
        gm.add_obstacle(250.0, 250.0, 30.0)
        path = plan_path(gm, (50.0, 50.0), (450.0, 450.0))
        assert len(path) > 2
        final = path[-1]
        assert math.hypot(
            final[0] - 450.0, final[1] - 450.0) < 10.0

    def test_aar_recording_and_replay(self):
        """AAR kayit-replay dongusu calismali."""
        records = [
            {
                'timestamp': 0.0,
                'topic': 'tracks',
                'data': {
                    'count': 1,
                    'tracks': [{
                        'track_id': 1, 'status': 1,
                        'classification': 'COMMERCIAL_QUAD',
                        'position': [1000, 500, 200],
                        'speed': 15.0,
                        'threat_score': 65.0,
                        'confidence': 0.8,
                    }],
                },
            },
            {
                'timestamp': 5.0,
                'topic': 'engagement',
                'data': {
                    'engagement_id': 1,
                    'track_id': 1,
                    'unit_id': 'alpha',
                    'effector_type': 0,
                    'result': 1,
                    'sensor_to_shoot_time': 3.0,
                    'miss_distance': 0.0,
                },
            },
        ]

        fd, path = tempfile.mkstemp(suffix='.jsonl')
        try:
            with os.fdopen(fd, 'w') as f:
                for r in records:
                    f.write(json.dumps(r) + '\n')

            loaded = load_recording(path)
            assert len(loaded) == 2

            engagements = extract_engagement_records(loaded)
            assert len(engagements) == 1

            snapshots = extract_track_snapshots(loaded)
            m = compute_track_metrics(snapshots)
            assert m['max_concurrent_tracks'] == 1

            report = generate_full_report(path)
            assert 'Angajman' in report
            assert '100.0%' in report
        finally:
            os.unlink(path)


class TestMonteCarloValidation:
    """Monte Carlo dogrulama testleri."""

    def test_fusion_lower_covariance(self):
        """Coklu sensor radardan daha dusuk kovaryans vermeli."""
        results = run_monte_carlo(n_runs=100, n_steps=60)
        radar_cov = results['radar_only']['mean_cov_trace']
        fused_cov = results['all_sensors']['mean_cov_trace']
        assert fused_cov < radar_cov

    def test_two_sensor_lower_covariance(self):
        """Iki sensor tek sensorden dusuk kovaryans vermeli."""
        results = run_monte_carlo(n_runs=100, n_steps=60)
        radar_cov = results['radar_only']['mean_cov_trace']
        re_cov = results['radar_eoir']['mean_cov_trace']
        assert re_cov < radar_cov
