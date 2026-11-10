"""Offline lane evaluation; never imports or starts CARLA."""
from __future__ import annotations
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from perception.lane_detection import LaneSteering, detect_lane, draw_overlay


def evaluate(root, split, overlays=0):
    rows = [json.loads(s) for s in (root / 'labels.jsonl').read_text().splitlines() if s.strip()]
    def key(r):
        sequence, drive, tick = Path(r['file']).stem.rsplit('_', 2)
        return sequence, int(drive), int(tick)
    rows.sort(key=key)
    groups = defaultdict(list)
    controller, previous = None, None
    saved = defaultdict(int)
    for row in rows:
        town, drive, tick = key(row)
        if drive not in ({0, 1} if split == 'tune' else {2}):
            continue
        if previous != (town, drive):
            controller, previous = LaneSteering(), (town, drive)
        if abs(row['offset_m']) > 1.5:
            continue
        image = cv2.imread(str(root / row['file']))
        if image is None:
            raise FileNotFoundError(root / row['file'])
        estimate = detect_lane(image, prior_width_px=controller.prior_width_px)
        steer = controller.update(estimate)
        detected = estimate.confidence > 0
        eligible = abs(row['offset_m']) > .2
        values = (detected,
                  abs(estimate.lateral_offset_m-row['offset_m']) if detected else np.nan,
                  math.degrees(abs(math.atan2(math.sin(estimate.heading_error_rad-row['heading_rad']), math.cos(estimate.heading_error_rad-row['heading_rad'])))) if detected else np.nan,
                  float(detected and estimate.lateral_offset_m*row['offset_m'] > 0) if eligible else np.nan,
                  float(steer*row['offset_m'] < 0) if eligible else np.nan)
        groups[row['town']].append(values)
        if saved[town] < overlays:
            dest = root / 'overlays' / split
            dest.mkdir(parents=True, exist_ok=True)
            overlay = draw_overlay(image, estimate)
            cv2.putText(overlay, f"truth={row['offset_m']:.3f}m steer={steer:.3f}", (10, 48), 0, .5, (0, 0, 255), 1)
            cv2.imwrite(str(dest / row['file']), overlay)
            saved[town] += 1
    groups['Overall'] = [v for town, values in groups.items() for v in values]
    lines = ['| Split | Town | N | Detection | Offset MAE (m) | Heading MAE (deg) | Offset sign | Steering sign |', '|---|---|---:|---:|---:|---:|---:|---:|']
    for town, values in groups.items():
        means = np.nanmean(np.asarray(values, dtype=float), axis=0)
        lines.append(f'| {split} | {town} | {len(values)} | {means[0]:.1%} | {means[1]:.3f} | {means[2]:.2f} | {means[3]:.1%} | {means[4]:.1%} |')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path(__file__).resolve().parents[2] / 'data/lane')
    parser.add_argument('--split', choices=['tune', 'test', 'all'], default='all')
    parser.add_argument('--save-overlays', type=int, default=0)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    tables = [evaluate(args.dataset, split, args.save_overlays) for split in (['tune', 'test'] if args.split == 'all' else [args.split])]
    report = '# Offline lane evaluation\n\nExclude |true offset| > 1.5 m. MAEs use detected frames. Both sign metrics use |true offset| > 0.2 m; missed detections count as wrong offset signs. Steering includes loss decay. Controller resets per town/drive; retained frames run in tick order. Overall pools frames.\n\n' + '\n\n'.join(tables) + '\n'
    (args.dataset / 'eval.md').write_text(report, encoding='utf-8')
    print(report)

if __name__ == '__main__':
    main()
