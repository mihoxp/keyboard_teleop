#!/usr/bin/env python3
"""Drive car 2 with the YAHBOOM USB 2.4G wireless gamepad (DragonRise 0079:181c).

Reads the joystick device straight from /dev/input and publishes /car2/cmd_vel
while a stick is pushed:

  left stick up/down     forward / backward (proportional)
  right stick left/right turn (proportional; both sticks combine into a curve)
  D-pad                  drive while held at the set speed, like the keyboard arrows
  X                      stop
  START                  horn while held
  L1 or L2 / R1 or R2    speed down / up: 30 / 50 / 75 / 100 %  (beep pitch = speed,
                         low buzz = already slowest / fastest)
  triangle / circle / square / SELECT   line_track / avoid / light_follow / manual
"""
import os
import select
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt16MultiArray

DEVICE = '/dev/input/by-id/usb-0079_Controller-joystick'

# struct js_event: time (ms), value, type, number
EVENT = struct.Struct('IhBB')
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80

# Mapped by pressing one control at a time 2026-10-24
AXIS_THROTTLE = 1   # left stick Y, up = -32767
AXIS_STEER = 2      # right stick X, right = +32767
AXIS_DPAD_X, AXIS_DPAD_Y = 6, 7     # digital: -32767 / 0 / +32767, up = -
BTN_CROSS, BTN_CIRCLE, BTN_SQUARE, BTN_TRIANGLE = 0, 1, 3, 4
BTN_L1, BTN_R1, BTN_L2, BTN_R2, BTN_SELECT, BTN_START = 6, 7, 8, 9, 10, 11
# L2/R2 also move axes 5/4 (unused)
SPEED_DOWN, SPEED_UP = (BTN_L1, BTN_L2), (BTN_R1, BTN_R2)
MODES = {BTN_TRIANGLE: 'line_track', BTN_CIRCLE: 'avoid',
         BTN_SQUARE: 'light_follow', BTN_SELECT: 'manual'}

# Same scale as keyboard_drive.py / the firmware (motor_config.h)
MAX_LINEAR = 0.5
MAX_ANGULAR = 2 * 0.5 / 0.10
SPEEDS = [0.3, 0.5, 0.75, 1.0]
# Far apart so each speed is easy to tell by ear; a low buzz = already at the limit
SPEED_BEEPS = [500, 1000, 2000, 3500]
LIMIT_BEEP = 200
# Horn: each beep lasts longer than the refresh period, so it sounds continuous
# while held and still stops by itself if the release message is lost
HORN_HZ, HORN_MS, HORN_PERIOD = 800, 300, 0.2
PUBLISH_PERIOD = 0.1    # the firmware stops after 0.5 s without cmd_vel
DEADZONE = 0.15         # pressing buttons nudges the sticks a little


def stick(value):
    x = value / 32767
    if abs(x) < DEADZONE:
        return 0.0
    # Rescale so the output starts at 0 just outside the dead zone
    return (abs(x) - DEADZONE) / (1 - DEADZONE) * (1 if x > 0 else -1)


def main():
    rclpy.init()
    node = rclpy.create_node('car2_gamepad_drive')
    cmd_pub = node.create_publisher(Twist, '/car2/cmd_vel', 10)
    mode_pub = node.create_publisher(String, '/car2/mode', 10)
    beep_pub = node.create_publisher(UInt16MultiArray, '/car2/buzzer', 10)
    log = node.get_logger()

    axes = {AXIS_THROTTLE: 0, AXIS_STEER: 0, AXIS_DPAD_X: 0, AXIS_DPAD_Y: 0}
    speed = 1   # index into SPEEDS
    was_moving = False
    horn = False
    next_horn = 0.0
    fd = None

    def publish_drive():
        frac = SPEEDS[speed]
        # The stick wins while it is pushed; otherwise use the D-pad
        throttle = stick(axes[AXIS_THROTTLE]) or stick(axes[AXIS_DPAD_Y])
        steer = stick(axes[AXIS_STEER]) or stick(axes[AXIS_DPAD_X])
        linear = -throttle * frac * MAX_LINEAR
        angular = -steer * frac * MAX_ANGULAR
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        cmd_pub.publish(msg)
        return linear != 0 or angular != 0

    def stop():
        nonlocal was_moving
        # cmd_vel is best effort over WiFi: send the stop a few times
        msg = Twist()
        for _ in range(3):
            cmd_pub.publish(msg)
        was_moving = False

    def beep(freq, ms=80):
        beep_pub.publish(UInt16MultiArray(data=[freq, ms]))

    log.info('gamepad drive ready')
    next_publish = time.monotonic()
    while rclpy.ok():
        if fd is None:
            try:
                fd = os.open(DEVICE, os.O_RDONLY | os.O_NONBLOCK)
                log.info(f'opened {DEVICE}')
            except OSError as e:
                log.warn(f'cannot open gamepad ({e.strerror}), retrying', throttle_duration_sec=30)
                time.sleep(2)
                continue

        ready, _, _ = select.select([fd], [], [], PUBLISH_PERIOD)
        if ready:
            try:
                data = os.read(fd, EVENT.size * 64)
            except OSError:
                # Receiver unplugged: never keep driving
                log.warn('gamepad lost')
                os.close(fd)
                fd = None
                for a in axes:
                    axes[a] = 0
                horn = False
                if was_moving:
                    stop()
                continue
            for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
                _, value, etype, number = EVENT.unpack_from(data, i)
                init = etype & JS_EVENT_INIT
                etype &= ~JS_EVENT_INIT
                if etype == JS_EVENT_AXIS:
                    if number in axes:
                        axes[number] = value
                        next_publish = 0.0
                    continue
                if etype != JS_EVENT_BUTTON or init:
                    continue
                if number == BTN_START:
                    horn = bool(value)
                    next_horn = 0.0
                    if not horn:
                        beep(0, 0)
                    continue
                # Other buttons act on presses only
                if value != 1:
                    continue
                if number == BTN_CROSS:
                    stop()
                elif number in SPEED_DOWN + SPEED_UP:
                    step = 1 if number in SPEED_UP else -1
                    new_speed = max(0, min(len(SPEEDS) - 1, speed + step))
                    if new_speed == speed:
                        beep(LIMIT_BEEP, 250)
                        continue
                    speed = new_speed
                    beep(SPEED_BEEPS[speed], 150)
                    log.info(f'speed {int(SPEEDS[speed] * 100)}%')
                elif number in MODES:
                    # A stop after this would switch the car back to manual
                    was_moving = False
                    mode_pub.publish(String(data=MODES[number]))
                    log.info(f'mode {MODES[number]}')

        now = time.monotonic()
        if horn and now >= next_horn:
            next_horn = now + HORN_PERIOD
            beep(HORN_HZ, HORN_MS)
        if now >= next_publish:
            next_publish = now + PUBLISH_PERIOD
            moving = publish_drive() if any(stick(v) for v in axes.values()) else False
            if was_moving and not moving:
                stop()
            was_moving = moving

    if was_moving:
        stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
