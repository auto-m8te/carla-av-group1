"""Join same-format MP4 clips with one-second black separators."""
import argparse
from pathlib import Path

import cv2
import numpy as np


def make_demo_reel(inputs, out):
    if not inputs:
        raise ValueError("At least one input is required")
    out = Path(out)
    if any(Path(p).resolve() == out.resolve() for p in inputs):
        raise ValueError("Output must differ from input files")
    writer = None
    expected = None
    try:
        for index, path in enumerate(inputs):
            capture = cv2.VideoCapture(str(path))
            try:
                if not capture.isOpened():
                    raise ValueError(f"Could not open input: {path}")
                fps = capture.get(cv2.CAP_PROP_FPS)
                size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
                if fps <= 0 or min(size) <= 0:
                    raise ValueError(f"Invalid video format: {path}")
                if expected is None:
                    expected = (fps, size)
                    out.parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
                    if not writer.isOpened():
                        raise RuntimeError(f"Could not open output: {out}")
                elif size != expected[1] or abs(fps - expected[0]) > .01:
                    raise ValueError("All input videos must have the same size and fps")
                if index:
                    black = np.zeros((size[1], size[0], 3), dtype=np.uint8)
                    for _ in range(round(fps)):
                        writer.write(black)
                frames = 0
                while True:
                    ok, frame = capture.read()
                    if not ok:
                        break
                    writer.write(frame)
                    frames += 1
                if not frames:
                    raise ValueError(f"Input has no decodable frames: {path}")
            finally:
                capture.release()
    finally:
        if writer is not None:
            writer.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', nargs='+', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    make_demo_reel(args.inputs, args.out)


if __name__ == '__main__':
    main()
