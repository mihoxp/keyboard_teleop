#!/usr/bin/env python3
"""Drive car 2 with the Pi's USB wireless keyboard, no screen or terminal needed.

Reads key events straight from /dev/input (needs the 'input' group) and
publishes /car2/cmd_vel while a drive key is held:

  arrows / WASD   drive (hold; two keys combine, e.g. up + left = curve left)
  space           stop
  + / -           speed 30 / 50 / 75 / 100 %  (beep pitch = speed)
  1 / 2 / 3 / 0   line_track / avoid / light_follow / manual
"""
import os
import select
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt16MultiArray

DEVICE = '/dev/input/by-id/usb-_mini_keyboard-event-kbd'

# struct input_event on 64-bit Linux: timeval (2 x long), type, code, value
EVENT = struct.Struct('llHHi')
EV_KEY = 1

KEY_1, KEY_2, KEY_3, KEY_0 = 2, 3, 4, 11
KEY_MINUS, KEY_EQUAL = 12, 13
KEY_W, KEY_A, KEY_S, KEY_D = 17, 30, 31, 32
KEY_SPACE = 57
KEY_KPMINUS, KEY_KPPLUS = 74, 78
KEY_UP, KEY_LEFT, KEY_RIGHT, KEY_DOWN = 103, 105, 106, 108

FORWARD = {KEY_UP, KEY_W}
BACKWARD = {KEY_DOWN, KEY_S}
LEFT = {KEY_LEFT, KEY_A}
RIGHT = {KEY_RIGHT, KEY_D}
DRIVE_KEYS = FORWARD | BACKWARD | LEFT | RIGHT
MODES = {KEY_1: 'line_track', KEY_2: 'avoid', KEY_3: 'light_follow', KEY_0: 'manual'}

# Same scale as the firmware (motor_config.h): full speed = 0.5 m/s per wheel,
# full turn = wheels at full speed in opposite directions
MAX_LINEAR = 0.5
MAX_ANGULAR = 2 * 0.5 / 0.10
SPEEDS = [0.3, 0.5, 0.75, 1.0]
PUBLISH_PERIOD = 0.1    # the firmware stops after 0.5 s without cmd_vel
# Held keys auto-repeat every ~33 ms; if a wireless release event is lost,
# this many seconds without any event counts as released
KEY_TIMEOUT = 0.6


def main():
    rclpy.init()
    node = rclpy.create_node('car2_keyboard_drive')
    cmd_pub = node.create_publisher(Twist, '/car2/cmd_vel', 10)
    mode_pub = node.create_publisher(String, '/car2/mode', 10)
    beep_pub = node.create_publisher(UInt16MultiArray, '/car2/buzzer', 10)
    log = node.get_logger()

    held = set()
    speed = 1   # index into SPEEDS
    last_event = 0.0
    was_moving = False
    fd = None

    def publish_drive():
        frac = SPEEDS[speed]
        linear = (bool(held & FORWARD) - bool(held & BACKWARD)) * frac * MAX_LINEAR
        angular = (bool(held & LEFT) - bool(held & RIGHT)) * frac * MAX_ANGULAR
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        cmd_pub.publish(msg)
        return linear != 0 or angular != 0

    def stop():
        nonlocal was_moving
        held.clear()
        # cmd_vel is best effort over WiFi: send the stop a few times
        for _ in range(3):
            publish_drive()
        was_moving = False

    def beep(freq, ms=80):
        beep_pub.publish(UInt16MultiArray(data=[freq, ms]))

    log.info('keyboard drive ready')
    next_publish = time.monotonic()
    while rclpy.ok():
        if fd is None:
            try:
                fd = os.open(DEVICE, os.O_RDONLY | os.O_NONBLOCK)
                log.info(f'opened {DEVICE}')
            except OSError as e:
                log.warn(f'cannot open keyboard ({e.strerror}), retrying', throttle_duration_sec=30)
                time.sleep(2)
                continue

        ready, _, _ = select.select([fd], [], [], PUBLISH_PERIOD)
        if ready:
            try:
                data = os.read(fd, EVENT.size * 64)
            except OSError:
                # Receiver unplugged or keyboard gone: never keep driving
                log.warn('keyboard lost')
                os.close(fd)
                fd = None
                if was_moving:
                    stop()
                continue
            for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
                _, _, etype, code, value = EVENT.unpack_from(data, i)
                if etype != EV_KEY:
                    continue
                last_event = time.monotonic()
                if code in DRIVE_KEYS:
                    if value:       # 1 = press, 2 = auto-repeat
                        held.add(code)
                    else:
                        held.discard(code)
                    next_publish = 0.0
                elif value != 1:
                    continue
                elif code == KEY_SPACE:
                    stop()
                elif code in (KEY_EQUAL, KEY_KPPLUS, KEY_MINUS, KEY_KPMINUS):
                    step = 1 if code in (KEY_EQUAL, KEY_KPPLUS) else -1
                    speed = max(0, min(len(SPEEDS) - 1, speed + step))
                    beep(800 + 400 * speed)
                    log.info(f'speed {int(SPEEDS[speed] * 100)}%')
                elif code in MODES:
                    # A stop after this would switch the car back to manual
                    held.clear()
                    was_moving = False
                    mode_pub.publish(String(data=MODES[code]))
                    log.info(f'mode {MODES[code]}')

        now = time.monotonic()
        if held and now - last_event > KEY_TIMEOUT:
            log.warn('no key events, assuming keys released')
            held.clear()
        if held or was_moving:
            if now >= next_publish:
                next_publish = now + PUBLISH_PERIOD
                moving = publish_drive()
                if was_moving and not moving:
                    stop()
                was_moving = moving

    if was_moving:
        stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
