"""Sounds: the indicator tick, and sirens that get louder as they come closer.

CARLA itself makes no sound, so this node makes its own with pygame,
from the same topics the rest of the car uses.
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

RATE = 44100
TICK_PERIOD = 0.38     # s between indicator clicks


def stereo(wave):
    pcm = (np.clip(wave, -1.0, 1.0) * 32000).astype(np.int16)
    return np.ascontiguousarray(np.stack([pcm, pcm], axis=1))


def click(pitch):
    t = np.arange(int(0.025 * RATE)) / RATE
    return stereo(np.sin(2 * np.pi * pitch * t) * np.exp(-t * 300) * 0.8)


def wail():
    """Ambulance: a slow rise and fall, 4 s per sweep."""
    t = np.arange(int(4.0 * RATE)) / RATE
    freq = 950 + 450 * np.sin(2 * np.pi * t / 4.0 - np.pi / 2)
    return stereo(0.5 * np.sin(2 * np.pi * np.cumsum(freq) / RATE))


def yelp():
    """Police: fast sweeps, about four per second."""
    t = np.arange(int(2.0 * RATE)) / RATE
    freq = 900 + 500 * np.abs((t * 4.0) % 2.0 - 1.0)
    return stereo(0.5 * np.sign(np.sin(2 * np.pi * np.cumsum(freq) / RATE)) * 0.6)


class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        self.blinker = 'none'
        self.emergency = (float('inf'), 0.0)
        self.tick_on = False
        try:
            import pygame
            pygame.mixer.init(frequency=RATE, size=-16, channels=2)
            self.sounds = {
                'tick': pygame.sndarray.make_sound(click(2200)),
                'tock': pygame.sndarray.make_sound(click(1600)),
                1.0: pygame.sndarray.make_sound(wail()),
                2.0: pygame.sndarray.make_sound(yelp()),
                3.0: pygame.sndarray.make_sound(wail()),
            }
            self.siren_channel = pygame.mixer.Channel(0)
            self.siren_kind = 0.0
            self.ok = True
        except Exception as e:   # no sound card, or no pygame
            self.get_logger().warn(f'No sound: {e}')
            self.ok = False

        self.create_subscription(String, '/lanebot/lights', self.on_lights, 10)
        self.create_subscription(Float32MultiArray, '/lanebot/emergency', self.on_emergency, 10)
        self.create_timer(TICK_PERIOD, self.indicator)
        self.create_timer(0.1, self.siren)

    def on_lights(self, msg):
        self.blinker = msg.data.split()[0]

    def on_emergency(self, msg):
        self.emergency = (msg.data[0], msg.data[1])

    def indicator(self):
        if self.ok and self.blinker in ('left', 'right'):
            self.sounds['tick' if self.tick_on else 'tock'].play()
            self.tick_on = not self.tick_on

    def siren(self):
        if not self.ok:
            return
        distance, kind = self.emergency
        if kind and abs(distance) < 150.0:
            if self.siren_kind != kind or not self.siren_channel.get_busy():
                self.siren_channel.play(self.sounds[kind], loops=-1)
                self.siren_kind = kind
            self.siren_channel.set_volume(min(1.0, 20.0 / max(abs(distance), 5.0)))
        elif self.siren_channel.get_busy():
            self.siren_channel.fadeout(800)
            self.siren_kind = 0.0


def main():
    rclpy.init()
    node = AudioNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
