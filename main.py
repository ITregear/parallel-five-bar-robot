import serial
import numpy as np
import keyboard


def serial_message(angle_1, angle_2, z_en, serial_port):
    # Function to send a message to the Arduino, for it to then control the O-Drive
    # Sends four variables, in the following format:
    # <speed, direction, enable, calibration>
    # '<>' markers are crucial
    # Speed is a float, direction must only be 1 or -1, and enable and calibration are boolean
    string_to_send = "<" + str(float(angle_1)) \
                     + ", " + str(int(angle_2)) \
                     + ", " + str(int(z_en)) + "> \n"

    serial_port.write(string_to_send.encode('UTF-8'))


def keyboard_inputs():
    x_pos = 0
    y_pos = 300
    z_en = 0

    if keyboard.is_pressed('a'):
        x_pos = 100
        y_pos = 300

    if keyboard.is_pressed('s'):
        x_pos = 0
        y_pos = 250

    if keyboard.is_pressed('d'):
        x_pos = -100
        y_pos = 300

    if keyboard.is_pressed('w'):
        x_pos = 0
        y_pos = 400

    if keyboard.is_pressed('h'):
        z_en = 1
    if keyboard.is_pressed('b'):
        z_en = 0

    return x_pos, y_pos, z_en


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

    gamma1 = np.degrees(3 * np.pi/2 - (theta1 + alpha1))
    gamma2 = np.degrees(3 * np.pi/2 - (theta2 + alpha2))

    return gamma1, gamma2


def main():

    arduino_port = "COM3"  # Default COM Port
    baud = 115200  # Baud Rate - do not change, unless Arduino Serial.begin(115200) is changed too
    ser = serial.Serial(arduino_port, baud)

    steps = 200
    gear_ratio = 4

    while True:
        try:
            while ser.in_waiting > 30:  # Flush serial buffer, so data is most recent
                ser.readline()  # This reduces number of successfully read bytes, but ensures data is most recent

            get_data = ser.readline().decode('UTF-8', errors='ignore')  # Receive data from serial port
            data = get_data[0:][:-2]  # Used to ignore new line
            data_array = np.fromstring(data, dtype='float', count=4, sep=', ')

            true_step_1 = data_array[0]
            true_step_2 = data_array[1]
            limit_1 = data_array[2]
            limit_2 = data_array[3]

        except KeyboardInterrupt:  # End connection with serial port if ctrl+C pressed
            print("Keyboard Interrupt")

        print(true_step_1, true_step_2)

        x, y, z = keyboard_inputs()

        angle1, angle2 = inverse_kinematics(x, y)

        deg_to_steps = (steps * gear_ratio) / 360

        step1 = angle1 * deg_to_steps
        step2 = angle2 * deg_to_steps

        serial_message(angle1, angle2, z, ser)


if __name__ == "__main__":
    main()
