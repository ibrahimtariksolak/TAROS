#!/usr/bin/env python3
"""Monte Carlo fuzyon karsilastirmasini calistirir ve sonuclari raporlar."""

import argparse
import sys

from taros_tracker.monte_carlo import format_report, run_monte_carlo


def main():
    """Ana fonksiyon."""
    parser = argparse.ArgumentParser(
        description='TAROS Monte Carlo sensor fuzyon karsilastirmasi')
    parser.add_argument(
        '-n', '--runs', type=int, default=200,
        help='Monte Carlo deneme sayisi (varsayilan: 200)')
    parser.add_argument(
        '-s', '--steps', type=int, default=60,
        help='Yorunge adim sayisi (varsayilan: 60)')
    parser.add_argument(
        '--seed', type=int, default=42,
        help='Rasgele tohum (varsayilan: 42)')
    args = parser.parse_args()

    print(f'Monte Carlo baslatiliyor: {args.runs} deneme, '
          f'{args.steps} adim, tohum={args.seed}')
    print()

    results = run_monte_carlo(
        n_runs=args.runs,
        n_steps=args.steps,
        seed=args.seed,
    )

    print(format_report(results))
    print()

    print('Detayli sonuclar:')
    for name, r in results.items():
        print(f'  {name}:')
        print(f'    RMSE konum: {r["rmse_position"]:.2f} m')
        print(f'    RMSE hiz: {r["rmse_velocity"]:.2f} m/s')
        print(f'    Kovaryans izi: {r["mean_cov_trace"]:.1f}')
        print(f'    Son hata ort: '
              f'{r["final_pos_error_mean"]:.2f} +/- '
              f'{r["final_pos_error_std"]:.2f} m')

    return 0


if __name__ == '__main__':
    sys.exit(main())
