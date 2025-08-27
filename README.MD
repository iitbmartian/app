# ROS2 Multi-Tab Viewer & Publisher

A comprehensive PyQt6-based GUI application for visualizing ROS2 sensor data and publishing control commands.

## Features

- **Multi-Camera Image Viewer**: Display up to 5 camera feeds simultaneously (supports both raw and compressed images)
- **3D PointCloud Visualization**: Interactive 3D viewer with customizable rendering options
- **Data Publishers**: Send Twist commands, odometry data, and other ROS2 messages with joystick controls
- **Real-time Topic Monitor**: Live monitoring of ROS2 topics with auto-discovery
- **Responsive UI**: Fullscreen interface with organized tabs and real-time statistics

## Setup

```bash
git clone https://github.com/iitbmartian/app.git
cd app

python -m venv venv
source venv/bin/activate

pip install -r requirements.txt
```

## Run

```bash
source /opt/ros/jazzy/setup.bash # or zsh
python3 main.py
```

## Requirements

- ROS2 Jazzy (or compatible distribution)
- Python 3.8+
- PyQt6
- OpenCV
- PyQtGraph
- NumPy

## Usage

1. Launch the application to see available ROS2 topics
2. Use the **Images** tab to connect multiple camera feeds
3. View 3D point clouds in the **PointCloud** tab
4. Publish control commands from the **Publishers** tab
5. Monitor live data streams in the **Monitor** tab

The interface automatically discovers available topics and provides intuitive controls for connecting to different data streams.
