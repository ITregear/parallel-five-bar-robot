import numpy as np
import matplotlib.pyplot as plt
import keyboard


def inverse_kinematics(x_pos, y_pos):

    l1a = 230
    l1b = 240
    l2a = 230
    l2b = 240
    x1 = 170/2

    alpha1 = np.arctan2(y_pos, (x1 + x_pos))
    alpha2 = np.arctan2(y_pos, (x1 - x_pos))

    z1 = y_pos / np.sin(alpha1)
    z2 = y_pos / np.sin(alpha2)

    theta1 = np.arccos((l1a**2 + z1**2 - l1b**2) / (2 * l1a * z1))
    theta2 = np.arccos((l2a**2 + z2**2 - l2b**2) / (2 * l2a * z2))

    gamma1 = np.degrees(alpha1 + theta1)
    gamma2 = np.degrees(np.pi - alpha2 - theta2)

    print(np.degrees(alpha1), np.degrees(theta1), gamma1)
    print(np.degrees(alpha2), np.degrees(theta2), gamma2)

    return gamma1, gamma2


x = 0
y = 300

gamma1, gamma2 = inverse_kinematics(x, y)

xn = [-85, -85 - 230*np.cos(np.radians(180 - gamma1)), x, 85 + 230*np.cos(np.radians(gamma2)), 85]
yn = [0, 230*np.sin(np.radians(180 - gamma1)), y, 230*np.sin(np.radians(gamma2)), 0]

l1a = np.sqrt((xn[1]-xn[0])**2 + (yn[1] - yn[0])**2)
l1b = np.sqrt((xn[2]-xn[1])**2 + (yn[2] - yn[1])**2)
l2a = np.sqrt((xn[3]-xn[2])**2 + (yn[3] - yn[2])**2)
l2b = np.sqrt((xn[4]-xn[3])**2 + (yn[4] - yn[3])**2)

print(l1a, l1b, l2a, l2b)

plt.plot(xn, yn)
plt.show()
