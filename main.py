import serial
import numpy as np
import keyboard
import time


class Motor:

    def __init__(self, home_angle):
        self.home_angle = home_angle
        self.limit_state = 0
        self.current_step = 0
        self.angle = 0


def serial_message(step_1, step_2, z_en, reset_flag,serial_port):
    # Function to send a message to the Arduino, for it to then control the O-Drive
    # Sends four variables, in the following format:
    # <speed, direction, enable, calibration>
    # '<>' markers are crucial
    # Speed is a float, direction must only be 1 or -1, and enable and calibration are boolean
    string_to_send = "<" + str(float(step_1)) \
                     + ", " + str(int(step_2)) \
                     + ", " + str(int(z_en)) \
                     + ", " + str(int(reset_flag)) + "> \n"

    serial_port.write(string_to_send.encode('UTF-8'))


def keyboard_inputs():
    x_pos = 0
    y_pos = 300
    z_en = 0

    if keyboard.is_pressed('d'):
        x_pos = 100
        y_pos = 300

    if keyboard.is_pressed('s'):
        x_pos = 0
        y_pos = 200

    if keyboard.is_pressed('a'):
        x_pos = -100
        y_pos = 300

    if keyboard.is_pressed('w'):
        x_pos = 0
        y_pos = 400

    if keyboard.is_pressed('h'):
        z_en = 1

    return x_pos, y_pos, z_en


def inverse_kinematics(x_pos, y_pos):

    # Need to add code that translates x_pos, y_pos of pen to x_end, y_end of robot

    gear_ratio = 4
    steps = 200
    micro_stepping = 8

    deg_to_step = gear_ratio * micro_stepping * steps / 360

    l1a = 230
    l1b = 240
    l2a = 230
    l2b = 240
    x1 = 170/2

    alpha1 = np.arctan2(y_pos, (x1 + x_pos))
    alpha2 = np.arctan2(y_pos, (x1 - x_pos))

    z1 = y_pos / np.sin(alpha1)
    z2 = y_pos / np.sin(alpha2)

    theta1 = np.arccos((l1a ** 2 + z1 ** 2 - l1b ** 2) / (2 * l1a * z1))
    theta2 = np.arccos((l2a ** 2 + z2 ** 2 - l2b ** 2) / (2 * l2a * z2))

    gamma1 = np.degrees(alpha1 + theta1)
    gamma2 = np.degrees(np.pi - alpha2 - theta2)

    steps1 = int(gamma1 * deg_to_step)
    steps2 = int(gamma2 * deg_to_step)

    return steps1, steps2


def homing_sequence(motor1, motor2, ser):
    home_pos_x = 0
    home_pos_y = 300

    if motor1.limit_state == 0 or motor2.limit_state == 0:

        if motor1.limit_state == 0:
            new_step_1 = motor1.current_step - 10
        else:
            new_step_1 = motor1.current_step

        if motor2.limit_state == 0:
            new_step_2 = motor2.current_step - 10
        else:
            new_step_2 = motor2.current_step

        serial_message(new_step_1, new_step_2, 1, 0, ser)

        return True

    else:
        serial_message(355, -1600, 0, 1, ser)

        step1, step2 = inverse_kinematics(home_pos_x, home_pos_y)  # End target

        serial_message(step1, step2, 1 , 0, ser)

        return False


def draw_from_txt(filename):

    x_file, y_file, z_file = [], [], []

    file = open(filename, 'r')

    for line in file:

        x_file += [float(line.split(',')[0])]
        y_file += [float(line.split(',')[1])]
        z_file += [float(line.split(',')[2])]

    return x_file, y_file, z_file


def issue_position_command(x, y, z, motor1, motor2, count, ser):
    step1, step2 = inverse_kinematics(x[count], y[count])

    print(count, step1, step2, motor1.current_step, motor2.current_step)

    if motor1.current_step == step1 or motor2.current_step == step2:
        if count == (len(x_array) - 1):
            count = 1
        else:
            count += 1

    serial_message(step1, step2, z[count], 0, ser)

    return count


def main():

    arduino_port = "COM3"  # Default COM Port
    baud = 115200  # Baud Rate - do not change, unless Arduino Serial.begin(115200) is changed too
    ser = serial.Serial(arduino_port, baud)
    print("Connected to", arduino_port)

    motor1 = Motor(20)
    motor2 = Motor(-90)

    homing_flag = True

    x, y, z = draw_from_txt("test_coords.txt")  # Arrays
    command_count = 0

    while True:
        try:
            while ser.in_waiting > 30:  # Flush serial buffer, so data is most recent
                ser.readline()  # This reduces number of successfully read bytes, but ensures data is most recent

            get_data = ser.readline().decode('UTF-8', errors='ignore')  # Receive data from serial port
            data = get_data[0:][:-2]  # Used to ignore new line
            data_array = np.fromstring(data, dtype='float', count=4, sep=', ')

            motor1.current_step = data_array[0]
            motor2.current_step = data_array[1]
            motor1.limit_state = data_array[2]
            motor2.limit_state = data_array[3]

        except KeyboardInterrupt:  # End connection with serial port if ctrl+C pressed
            print("Keyboard Interrupt")

        if keyboard.is_pressed('space') or homing_flag:
            homing_flag = homing_sequence(motor1, motor2, ser)
        else:
            # command_count = issue_position_command(x, y, z, motor1, motor2, command_count, ser)  # XYZ are arrays
            x, y, z = keyboard_inputs()
            step1, step2 = inverse_kinematics(x, y)
            serial_message(step1, step2, z, 0, ser)

        print(motor1.current_step, motor2.current_step)


if __name__ == "__main__":
    main()
