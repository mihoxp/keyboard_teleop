#!/usr/bin/env python3
"""Drive car 2 with the YAHBOOM USB 2.4G wireless gamepad (DragonRise 0079:181c).

Reads the joystick device straight from /dev/input and publishes /car2/cmd_vel
while a stick is pushed:

  left stick             up/down = forward / backward, left/right = turn
                         (proportional, combine into a curve)
  right stick            left/right = servo S1, up/down = servo S2: full push =
                         0 / 180 deg, released = back to 90
  L1 / R1                hold to turn servo S3 toward 0 / 180 deg, it stays where
                         released; L1 + R1 together = back to 90
  D-pad                  drive while held at the set speed, like the keyboard arrows
  X                      stop
  START                  horn while held
  left stick press (L3)  auto lights on / off (turn signals, head / brake lights)
  right stick press (R3) all 8 LEDs white on / off
  L2 / R2                speed down / up: 30 / 50 / 75 / 100 %  (beep pitch = speed,
                         low buzz = already slowest / fastest)
  triangle / circle / square / SELECT   line_track / avoid / light_follow / manual
"""
import os
import select
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, UInt16MultiArray, UInt32MultiArray

DEVICE = '/dev/input/by-id/usb-0079_Controller-joystick'

# struct js_event: time (ms), value, type, number
EVENT = struct.Struct('IhBB')
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80

# Mapped by pressing one control at a time 2026-10-24
AXIS_THROTTLE = 1   # left stick Y, up = -32767
AXIS_STEER = 0      # left stick X, right = +32767
AXIS_SERVO1 = 2     # right stick X, right = +32767
AXIS_SERVO2 = 3     # right stick Y, up = -32767
AXIS_DPAD_X, AXIS_DPAD_Y = 6, 7     # digital: -32767 / 0 / +32767, up = -
BTN_CROSS, BTN_CIRCLE, BTN_SQUARE, BTN_TRIANGLE = 0, 1, 3, 4
BTN_L1, BTN_R1, BTN_L2, BTN_R2, BTN_SELECT, BTN_START = 6, 7, 8, 9, 10, 11
BTN_L3, BTN_R3 = 13, 14   # stick presses
# L2/R2 also move axes 5/4 (unused)
SPEED_DOWN, SPEED_UP = (BTN_L2,), (BTN_R2,)
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
# Half brightness: all 8 LEDs at full white draw ~0.5 A from the battery
WHITE = 0x808080
LED_COUNT = 8
PUBLISH_PERIOD = 0.1    # the firmware stops after 0.5 s without cmd_vel
DEADZONE = 0.15         # pressing buttons nudges the sticks a little
SERVO_PERIOD = 0.05     # max servo update rate while the right stick / L1 / R1 moves
SERVO3_RATE = 90.0      # deg/s while L1 / R1 is held


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
    auto_lights_pub = node.create_publisher(Bool, '/car2/auto_lights', 10)
    leds_pub = node.create_publisher(UInt32MultiArray, '/car2/leds', 10)
    servo_pub = node.create_publisher(UInt16MultiArray, '/car2/servo', 10)
    log = node.get_logger()

    axes = {AXIS_THROTTLE: 0, AXIS_STEER: 0, AXIS_DPAD_X: 0, AXIS_DPAD_Y: 0}
    speed = 1   # index into SPEEDS
    was_moving = False
    horn = False
    next_horn = 0.0
    # 'auto' / 'white' / 'off'; the firmware boots with auto lights on
    lights = 'auto'
    servo_axes = {AXIS_SERVO1: 0, AXIS_SERVO2: 0}
    sent_servo = None   # last [S1, S2] sent
    next_servo = 0.0
    # Assumed until the first move: the car does not report servo angles
    servo3_angle = 90.0
    servo3_held = set()
    last_servo3 = 0.0
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

    def publish_servo3():
        # /car2/servo is [S1, S2, S3]: repeat what the right stick last sent
        s1, s2 = sent_servo if sent_servo else (90, 90)
        servo_pub.publish(UInt16MultiArray(data=[s1, s2, round(servo3_angle)]))

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

        timeout = SERVO_PERIOD if servo3_held else PUBLISH_PERIOD
        ready, _, _ = select.select([fd], [], [], timeout)
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
                for a in servo_axes:
                    servo_axes[a] = 0
                servo3_held.clear()
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
                    elif number in servo_axes:
                        servo_axes[number] = value
                    continue
                if etype != JS_EVENT_BUTTON or init:
                    continue
                if number == BTN_START:
                    horn = bool(value)
                    next_horn = 0.0
                    if not horn:
                        beep(0, 0)
                    continue
                if number in (BTN_L1, BTN_R1):
                    if value:
                        servo3_held.add(number)
                    else:
                        servo3_held.discard(number)
                    if servo3_held == {BTN_L1, BTN_R1}:
                        servo3_angle = 90.0
                        publish_servo3()
                    last_servo3 = time.monotonic()
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
                elif number == BTN_L3:
                    # Any /car2/leds message (white) already turned auto off
                    lights = 'off' if lights == 'auto' else 'auto'
                    auto_lights_pub.publish(Bool(data=lights == 'auto'))
                    beep(1500 if lights == 'auto' else 600)
                    log.info(f'lights {lights}')
                elif number == BTN_R3:
                    lights = 'off' if lights == 'white' else 'white'
                    color = WHITE if lights == 'white' else 0
                    leds_pub.publish(UInt32MultiArray(data=[color] * LED_COUNT))
                    beep(1500 if lights == 'white' else 600)
                    log.info(f'lights {lights}')
                elif number in MODES:
                    # A stop after this would switch the car back to manual
                    was_moving = False
                    mode_pub.publish(String(data=MODES[number]))
                    log.info(f'mode {MODES[number]}')

        now = time.monotonic()
        if len(servo3_held) == 1 and now - last_servo3 >= SERVO_PERIOD:
            step = SERVO3_RATE * (now - last_servo3)
            last_servo3 = now
            old = round(servo3_angle)
            servo3_angle += step if BTN_R1 in servo3_held else -step
            servo3_angle = max(0.0, min(180.0, servo3_angle))
            if round(servo3_angle) != old:
                publish_servo3()
        if now >= next_servo:
            # Stick position = angle. Only changes are sent, so a centered
            # stick does not override /car2/servo from other sources
            angles = [round(90 + 90 * stick(servo_axes[AXIS_SERVO1])),
                      round(90 - 90 * stick(servo_axes[AXIS_SERVO2]))]
            if sent_servo is None:
                sent_servo = angles     # nothing to send until the stick moves
            elif angles != sent_servo:
                next_servo = now + SERVO_PERIOD
                servo_pub.publish(UInt16MultiArray(data=angles))
                sent_servo = angles
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
