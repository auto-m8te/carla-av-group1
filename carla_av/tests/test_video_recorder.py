from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from avoidance.collision_avoidance import Mode
from scenarios.video_recorder import VideoRecorder, draw_hud
from tools.make_demo_reel import make_demo_reel


def test_hud_modifies_blank_frame():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    result = draw_hud(frame, Mode.CRUISE, 10, None, {
        'scenario': 'obstacle_avoid', 'town': 'Town03', 'run': 1,
        'lane_source': 'camera', 'obstacle_source': 'lidar'})
    assert result.shape == (720, 1280, 3)
    assert np.count_nonzero(result) > 0


def read_frames(path):
    cap = cv2.VideoCapture(str(path))
    frames = []
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frames.append(frame)
    finally:
        cap.release()
    return frames


def test_reel_frame_count_and_separator(tmp_path):
    inputs = [tmp_path / 'a.mp4', tmp_path / 'b.mp4']
    for path, count in zip(inputs, (3, 4)):
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), 10, (64, 48))
        assert writer.isOpened()
        for _ in range(count):
            writer.write(np.full((48, 64, 3), 180, dtype=np.uint8))
        writer.release()
    out = tmp_path / 'reel.mp4'
    make_demo_reel(inputs, out)
    frames = read_frames(out)
    assert len(frames) == 3 + 10 + 4
    assert all(np.max(frame) < 5 for frame in frames[3:13])
    assert frames[0].shape == (48, 64, 3)


@pytest.mark.parametrize('fps', [10, 20])
def test_title_card_duration_and_close(monkeypatch, fps):
    recorder = VideoRecorder.__new__(VideoRecorder)
    recorder.world = object()
    recorder.fps, recorder.size = fps, (320, 180)
    recorder._closed, recorder._started = False, False
    recorder._writer = Mock()
    recorder._queue = object()
    image = SimpleNamespace(height=180, width=320,
                            raw_data=np.zeros((180, 320, 4), dtype=np.uint8).tobytes())
    monkeypatch.setattr('scenarios.video_recorder.current_frame', lambda world: 42)
    wait = Mock(return_value=image)
    monkeypatch.setattr('scenarios.video_recorder.wait_for_frame', wait)
    recorder.add_frame(Mode.CRUISE, 1, None, {'scenario': 'empty_lane'})
    assert recorder._writer.write.call_count == 2 * fps + 1
    wait.assert_called_once_with(recorder._queue, 42)
    recorder.add_frame(Mode.CRUISE, 1, None, {'scenario': 'empty_lane'})
    assert recorder._writer.write.call_count == 2 * fps + 2
    recorder.close()
    recorder.close()
    recorder._writer.release.assert_called_once()
