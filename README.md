# Parallel5BarRobot

This repository contains the code for controlling a robotic arm using inverse kinematics, interfacing with an Arduino for hardware control. The project is designed to interpret and execute movement commands based on pre-defined or real-time inputs.

## Files in the Repository

- **main.py:** The core script that includes the `Motor` class, functions for serial communication with the Arduino, keyboard input handling, inverse kinematics calculations, homing sequence, drawing from a text file, and the main execution loop.
- **inv_kin.py:** A script for visualizing the robotic arm's movements using matplotlib. It imports functions from `main.py` for inverse kinematics calculations and drawing coordinates from a text file.

## Requirements

- Python 3.x
- NumPy
- Matplotlib
- PySerial
- Keyboard

## Installation

1. Ensure that Python 3.x is installed on your system.
2. Install the required Python libraries using pip:

    ```bash
    pip install numpy matplotlib pyserial keyboard
    ```

3. Clone this repository or download the files to your local machine.

## Usage

### main.py

- Connect your Arduino to your computer and note the COM port it's connected to.
- Update the `arduino_port` variable in `main.py` to match the noted COM port.
- Run `main.py` to start the program:

    ```bash
    python main.py
    ```

- Use keyboard inputs to control the robotic arm in real-time or let it follow predefined paths from a text file.

### inv_kin.py

- Run `inv_kin.py` to visualize the robotic arm's movements based on the coordinates from "test_coords.txt":

    ```bash
    python inv_kin.py
    ```

## Features

- **Inverse Kinematics:** Converts desired end-effector positions into joint angles.
- **Real-time Control:** Uses keyboard inputs to control the arm in real-time.
- **Automated Movements:** Can follow predefined paths from a text file.
- **Visualization:** Provides a graphical representation of the arm's movement.

## Contributing

Contributions to this project are welcome. Please fork the repository and submit a pull request for any enhancements, bug fixes, or improvements.

## License

This project is open-source and available under the [GNU General Public License](LICENSE.md).
