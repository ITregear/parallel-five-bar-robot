import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import keyboard
from main import inverse_kinematics
from main import draw_from_txt
import time


def inv_kin():

    matplotlib.use("TkAgg")
    fig, ax = plt.subplots()
    ax.set_xlim(-300, 300)
    ax.set_ylim(0, 450)

    line1, = ax.plot([], [], c='r')
    line2, = ax.plot([], [], c='b')

    gear_ratio = 4
    steps = 200
    micro_stepping = 8

    l1a = 230
    l1b = 240
    l2a = 230
    l2b = 240
    x1 = 170 / 2

    delay = 0.1
    last_time = 0

    deg_to_step = gear_ratio * micro_stepping * steps / 360

    x_array, y_array, z_array = draw_from_txt("test_coords.txt")

    count = 0
    x_end = []
    y_end = []

    plt.ion()

    while True:
        try:

            if (time.perf_counter() - last_time) >= delay:
                last_time = time.perf_counter()
                if count == (len(x_array)-1):
                    count = 1
                else:
                    count += 1

                print(count)

                step1, step2 = inverse_kinematics(x_array[count], y_array[count])

                gamma1 = step1 / deg_to_step
                gamma2 = step2 / deg_to_step

                xn = [-x1, -x1 - l1a * np.cos(np.radians(180 - gamma1)), x_array[count], x1 + l2a * np.cos(np.radians(gamma2)), 85]
                yn = [0, l1a * np.sin(np.radians(180 - gamma1)), y_array[count], l2a * np.sin(np.radians(gamma2)), 0]

                x_end += [x_array[count]]
                y_end += [y_array[count]]

                line1.set_data(xn, yn)
                line2.set_data(x_end, y_end)

                plt.pause(0.05)

        except KeyboardInterrupt:
            print("Keyboard Interrupt")

    plt.show()


if __name__ == "__main__":
    inv_kin()
