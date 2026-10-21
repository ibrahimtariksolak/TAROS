"""Monte Carlo sensor fuzyon karsilastirmasi birim testleri."""

from taros_tracker.monte_carlo import (
    format_report,
    run_monte_carlo,
)


class TestMonteCarlo:
    """Monte Carlo simulasyon testleri."""

    def test_all_configs_present(self):
        """Tum sensor konfigurasyonlari sonucta yer almali."""
        results = run_monte_carlo(n_runs=5, n_steps=30)
        expected = {
            'radar_only', 'eoir_only', 'acoustic_only',
            'radar_eoir', 'radar_acoustic', 'all_sensors',
        }
        assert set(results.keys()) == expected

    def test_fused_better_than_single(self):
        """Coklu sensor tek sensorden daha iyi olmali."""
        results = run_monte_carlo(n_runs=100, n_steps=60)
        radar_cov = results['radar_only']['mean_cov_trace']
        fused_cov = results['all_sensors']['mean_cov_trace']
        assert fused_cov < radar_cov

    def test_more_sensors_lower_covariance(self):
        """Daha fazla sensor daha dusuk kovaryans izi vermeli."""
        results = run_monte_carlo(n_runs=100, n_steps=60)
        radar_cov = results['radar_only']['mean_cov_trace']
        re_cov = results['radar_eoir']['mean_cov_trace']
        assert re_cov < radar_cov

    def test_rmse_positive(self):
        """RMSE degerleri pozitif olmali."""
        results = run_monte_carlo(n_runs=5, n_steps=30)
        for cfg in results.values():
            assert cfg['rmse_position'] > 0
            assert cfg['rmse_velocity'] > 0

    def test_convergence_within_steps(self):
        """Yakin baslangicta yakinsama 30 adim icinde olmali."""
        results = run_monte_carlo(n_runs=10, n_steps=40)
        assert results['all_sensors']['convergence_step'] < 30

    def test_format_report_nonempty(self):
        """Rapor bos olmamali."""
        results = run_monte_carlo(n_runs=5, n_steps=30)
        report = format_report(results)
        assert len(report) > 100
        assert 'radar_only' in report
        assert 'all_sensors' in report
