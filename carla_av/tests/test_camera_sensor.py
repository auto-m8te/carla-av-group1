from types import SimpleNamespace
import numpy as np
import pytest
from perception.camera_sensor import CameraLaneModule
from test_lane_detection import road


class Sensor:
    is_alive = True
    def listen(self, callback): self.callback = callback
    def stop(self): pass
    def destroy(self): self.is_alive = False


def test_camera_queue_hold_debug_and_destroy(tmp_path):
    sensor = Sensor()
    attributes = {}
    bp = SimpleNamespace(set_attribute=lambda k, v: attributes.update({k: v}))
    world = SimpleNamespace(get_blueprint_library=lambda: SimpleNamespace(find=lambda _: bp),
                            spawn_actor=lambda *a, **kw: sensor)
    module = CameraLaneModule(world, object(), debug_dir=tmp_path, debug_every=1)
    assert module(None) == 0
    image = np.dstack((road(-40), np.full((450, 800), 255, dtype=np.uint8)))
    sensor.callback(SimpleNamespace(raw_data=image.tobytes(), height=450, width=800))
    steer = module(None)
    assert steer < 0
    assert module(None) == steer
    assert module.last_estimate.confidence == 1
    assert len(list(tmp_path.glob('*.png'))) == 1
    assert attributes['sensor_tick'] == '0.0'
    module.destroy()
    module.destroy()

