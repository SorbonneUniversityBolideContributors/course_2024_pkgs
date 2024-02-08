# ROS Parameter Tuning GUI

This is a Graphical User Interface (GUI) developed for fine-tuning ROS (Robot Operating System) parameters. The GUI provides an intuitive interface for adjusting various parameters related to ROS nodes and functionalities.

## Features

- User-friendly interface designed using Qt Designer.
- Supports adjustment of boolean, numeric, and combobox parameters.
- Auto-calibration functionality for RGB color calibration.
- Save and load parameter configurations to/from files using pickle.
- Alerts ROS nodes of parameter changes using a dedicated topic.

## Dependencies

- ROS (Robot Operating System)
- Python 3
- PySide6 (Qt for Python)
- OpenCV (cv2)
- NumPy
- rospy (ROS Python client library)

## Usage

1. Ensure that ROS is properly installed and configured on your system.
2. Install the necessary Python dependencies listed in the `requirements.txt` file.
3. Run the ROS master node.
4. Launch the GUI by running the `main_gui.py` script.
5. Use the GUI to adjust ROS parameters as needed.
6. Save parameter configurations to files for future use.
7. Perform auto-calibration for RGB color calibration as required.

## File Structure

- `main_gui.py`: Main Python script containing the GUI implementation.
- `ui_files/qtapp.ui`: UI file created using Qt Designer.
- `scripts/`: Directory containing additional scripts and modules.
- `params/`: Directory for storing parameter configuration files saved by the user.

## Configuration

- ROS parameters are organized into dictionaries based on their types and purposes.
- GUI elements are connected to corresponding ROS parameters and functions using PyQt signals and slots.
