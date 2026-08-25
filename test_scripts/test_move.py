import time
import sys
import termios
import tty
from adafruit_pca9685 import PCA9685
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000


motors = [(4, 5), (7, 6), (8, 9), (11, 10)]  # Motor 1  # Motor 2  # Motor 3  # Motor 4


speed_percent = 15


def set_motor(motor_index, forward_percent, backward_percent):
    ch_forward, ch_backward = motors[motor_index]

    def percent_to_pwm(percent):
        return int(max(0, min(100, percent)) / 100 * 0xFFFF)

    pca.channels[ch_forward].duty_cycle = percent_to_pwm(forward_percent)
    pca.channels[ch_backward].duty_cycle = percent_to_pwm(backward_percent)


def motor_backward():
    for i in range(4):
        set_motor(i, speed_percent, 0)


def motor_forward():
    for i in range(4):
        set_motor(i, 0, speed_percent)


def motor_stop():
    for i in range(4):
        set_motor(i, 0, 0)


def turn_rotate_right():
    set_motor(0, 0, speed_percent)
    set_motor(1, speed_percent, 0)
    set_motor(2, 0, speed_percent)
    set_motor(3, speed_percent, 0)


def turn_rotate_left():
    set_motor(0, speed_percent, 0)
    set_motor(1, 0, speed_percent)
    set_motor(2, speed_percent, 0)
    set_motor(3, 0, speed_percent)


def turn_direct_left():
    set_motor(0, 0, speed_percent)
    set_motor(1, speed_percent, 0)
    set_motor(2, speed_percent, 0)
    set_motor(3, 0, speed_percent)


def turn_direct_right():
    set_motor(0, speed_percent, 0)
    set_motor(1, 0, speed_percent)
    set_motor(2, 0, speed_percent)
    set_motor(3, speed_percent, 0)


def turn_forward_diagonal_left():
    set_motor(0, speed_percent, 0)
    set_motor(1, 0, 0)
    set_motor(2, 0, 0)
    set_motor(3, speed_percent, 0)


def turn_forward_diagonal_right():
    set_motor(0, 0, 0)
    set_motor(1, speed_percent, 0)
    set_motor(2, speed_percent, 0)
    set_motor(3, 0, 0)


def turn_backward_diagonal_left():
    set_motor(0, 0, speed_percent)
    set_motor(1, 0, 0)
    set_motor(2, 0, 0)
    set_motor(3, 0, speed_percent)


def turn_backward_diagonal_right():
    set_motor(0, 0, 0)
    set_motor(1, 0, speed_percent)
    set_motor(2, 0, speed_percent)
    set_motor(3, 0, 0)


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


try:
    print("Start script")
    while True:
        key = get_key()

        if key == "w":
            speed_percent = min(speed_percent + 5, 100)
            print(f"Speed: {speed_percent}%")
        elif key == "s":
            speed_percent = max(speed_percent - 5, 0)
            print(f"Speed: {speed_percent}%")

        if key in [
            "\x1b[A",
            "\x1b[B",
            "\x1b[C",
            "\x1b[D",
            ",",
            ".",
            "q",
            "a",
            "e",
            "d",
        ]:
            if key == "\x1b[A":
                motor_forward()
            elif key == "\x1b[B":
                motor_backward()
            elif key == "\x1b[D":
                turn_rotate_left()
            elif key == "\x1b[C":
                turn_rotate_right()
            elif key == ",":
                turn_direct_left()
            elif key == ".":
                turn_direct_right()
            elif key == "q":
                turn_forward_diagonal_right()
            elif key == "d":
                turn_backward_diagonal_right()
            elif key == "e":
                turn_forward_diagonal_left()
            elif key == "a":
                turn_backward_diagonal_left()
        else:
            motor_stop()

        if key == "x":
            print("Exiting...")
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

finally:
    motor_stop()
    for ch in range(16):
        pca.channels[ch].duty_cycle = 0
