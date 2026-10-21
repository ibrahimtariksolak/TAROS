#!/usr/bin/env python3
"""AAR kayit dosyasindan metrik raporu olusturur."""

import argparse
import glob
import os
import sys

from taros_aar.replay import generate_full_report


def main():
    """Ana fonksiyon."""
    parser = argparse.ArgumentParser(
        description='TAROS AAR rapor olusturucu')
    parser.add_argument(
        'filepath', nargs='?', default=None,
        help='JSONL kayit dosyasi yolu (bos birak = en son)')
    parser.add_argument(
        '-d', '--directory', default='/tmp/taros_aar',
        help='Kayit dizini (varsayilan: /tmp/taros_aar)')
    args = parser.parse_args()

    filepath = args.filepath
    if filepath is None:
        pattern = os.path.join(args.directory, 'aar_*.jsonl')
        files = sorted(glob.glob(pattern))
        if not files:
            print(f'Kayit dosyasi bulunamadi: {pattern}',
                  file=sys.stderr)
            return 1
        filepath = files[-1]
        print(f'En son kayit: {filepath}')
        print()

    if not os.path.isfile(filepath):
        print(f'Dosya bulunamadi: {filepath}',
              file=sys.stderr)
        return 1

    report = generate_full_report(filepath)
    print(report)
    return 0


if __name__ == '__main__':
    sys.exit(main())
