from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import framebuf
from time import sleep
from ulab import numpy as np


# ===== parameters =====
# constants
G = 1

m_1 = 1
m_2 = 1
m_3 = 1

dt = 0.05

# display size
WIDTH = 128
HEIGHT = 64

# index for placeholder trajectories
to_plot = [1, 2, 3, 5, 7, 9, 12, 15, 18, 22, 26, 30, 35, 40, 45]

# GPIO pin numbers
pir_pin = 28
scl_pin = 1
sda_pin = 0

# devices
i2c = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
oled = SSD1306_I2C(128, 64, i2c)
led = Pin("LED", Pin.OUT)

# ===== helper functions =====
def flash():
    """
    flash the screen
    """
    oled.fill(0)
    oled.show()


def accelerations(p1, p2, p3):
    """
    A function to calculate the derivatives of x, y, and z
    given 3 object and their locations according to Newton's laws

    """

    dv1 = -G * m_2 * (p1 - p2) / (
        np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 3
    ) - G * m_3 * (p1 - p3) / (
        np.sqrt((p1[0] - p3[0]) ** 2 + (p1[1] - p3[1]) ** 2) ** 3
    )

    dv2 = -G * m_3 * (p2 - p3) / (
        np.sqrt((p2[0] - p3[0]) ** 2 + (p2[1] - p3[1]) ** 2) ** 3
    ) - G * m_1 * (p2 - p1) / (
        np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 3
    )

    dv3 = -G * m_1 * (p3 - p1) / (
        np.sqrt((p3[0] - p1[0]) ** 2 + (p3[1] - p1[1]) ** 2) ** 3
    ) - G * m_2 * (p3 - p2) / (
        np.sqrt((p3[0] - p2[0]) ** 2 + (p3[1] - p2[1]) ** 2) ** 3
    )

    return dv1, dv2, dv3


def normalize_position(x, y):
    """
    scale the coordinate system to fit the display
    """
    return round(x / 2.4 * WIDTH + WIDTH / 2 - 1), round(y / 1.6 * HEIGHT + HEIGHT / 2)


def draw_square(x, y):
    """
    draw 2x2 a square
    """
    x, y = normalize_position(x, y)

    # draw only if not out of display bounds
    if x >= 0 and x < WIDTH and y >= 0 and y < HEIGHT:
        oled.hline(x - 2, y - 2, 5, 1)
        oled.hline(x - 2, y - 1, 5, 1)
        oled.hline(x - 2, y, 5, 1)
        oled.hline(x - 2, y + 1, 5, 1)
        oled.hline(x - 2, y + 2, 5, 1)


def draw_circle(x, y):
    """
    draw a 2x2 circle
    """
    x, y = normalize_position(x, y)

    # draw only if not out of display bounds
    if x >= 0 and x < WIDTH and y >= 0 and y < HEIGHT:
        oled.hline(x - 1, y - 2, 3, 1)
        oled.hline(x - 2, y - 1, 5, 1)
        oled.hline(x - 2, y, 5, 1)
        oled.hline(x - 2, y + 1, 5, 1)
        oled.hline(x - 1, y + 2, 3, 1)


def draw_triangle(x, y):
    """
    draw a 2x2 triangle
    """
    x, y = normalize_position(x, y)

    # draw only if not out of display bounds
    if x >= 0 and x < WIDTH and y >= 0 and y < HEIGHT:
        oled.hline(x, y - 2, 1, 1)
        oled.hline(x, y - 1, 1, 1)
        oled.hline(x - 1, y, 3, 1)
        oled.hline(x - 1, y + 1, 3, 1)
        oled.hline(x - 2, y + 2, 5, 1)


def draw_pixel(x, y):
    """
    draw a 1x1 pixel
    """
    x, y = normalize_position(x, y)

    # draw only if not out of display bounds
    if x >= 0 and x < WIDTH and y >= 0 and y < HEIGHT:
        oled.pixel(x, y, 1)


def draw_tail(tail):
    """
    draw trajectory
    """
    for i in to_plot:
        array = tail.T[i]
        if array[0] != -99999:
            draw_pixel(array[0], array[1])


if __name__ == "__main__":
    # flash led to indicate power on
    led.on()
    sleep(0.1)
    led.off()
    sleep(1)

    # simulation starts
    # initial positions
    p1, p2, p3 = np.array([-1, 0]), np.array([1, 0]), np.array([0, 0])

    # initial speeds
    v1, v2, v3 = (
        np.array([0.203492, 0.518113]),
        np.array([0.203492, 0.518113]),
        np.array([-0.406984, -1.036226]),
    )

    # placeholder for trajectories
    tail1, tail2, tail3 = (
        np.full([2, max(to_plot) + 1], -99999),
        np.full([2, max(to_plot) + 1], -99999),
        np.full([2, max(to_plot) + 1], -99999),
    )

    while True:
        # draw current position
        flash()
        draw_square(p1[0], p1[1])
        draw_circle(p2[0], p2[1])
        draw_triangle(p3[0], p3[1])
        draw_tail(tail1)
        draw_tail(tail2)
        draw_tail(tail3)
        oled.show()

        # calculate derivatives
        dv1, dv2, dv3 = accelerations(p1, p2, p3)

        # velocity at the next time point
        v1 = v1 + dv1 * dt
        v2 = v2 + dv2 * dt
        v3 = v3 + dv3 * dt

        # position at the next dt
        p1 = p1 + v1 * dt
        p2 = p2 + v2 * dt
        p3 = p3 + v3 * dt

        # remember the current positions to plot the trajectories
        tail1 = np.roll(tail1, 1)
        tail2 = np.roll(tail2, 1)
        tail3 = np.roll(tail3, 1)
        tail1[:, 0] = p1
        tail2[:, 0] = p2
        tail3[:, 0] = p3
