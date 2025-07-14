import numpy as np
from PyQt6.QtWidgets import (QHBoxLayout, QVBoxLayout, QWidget, QGroupBox, QLabel, 
                            QComboBox, QPushButton, QPlainTextEdit, QGridLayout,
                            QCheckBox, QSlider, QSpinBox, QFrame)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from datetime import datetime
from sensor_msgs_py import point_cloud2
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class PointCloudTab(QWidget):
    def __init__(self, parent_signals, main_window):
        super().__init__()
        self.signals = parent_signals
        self.main_window = main_window
        self.signals.pointcloud_received.connect(self.update_pointcloud)
        self.available_topics = []
        self.connected_topic = None
        self.current_pointcloud = None
        self.raw_points = None
        self._camera_adjusted = False
        self.setup_ui()
    
    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        
        # Left panel for controls and stats
        left_panel = QWidget()
        left_panel.setMaximumWidth(350)
        left_layout = QVBoxLayout(left_panel)
        
        # Connection controls
        control_group = QGroupBox("PointCloud Connection")
        control_layout = QVBoxLayout(control_group)
        
        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("Topic:"))
        self.topic_combo = QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.setPlaceholderText("No pointcloud topics available")
        topic_layout.addWidget(self.topic_combo)
        control_layout.addLayout(topic_layout)
        
        button_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_pointcloud)
        self.connect_btn.setEnabled(False)
        button_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_pointcloud)
        self.disconnect_btn.setEnabled(False)
        button_layout.addWidget(self.disconnect_btn)
        control_layout.addLayout(button_layout)
        
        left_layout.addWidget(control_group)
        
        # Visualization controls
        viz_control_group = QGroupBox("Visualization Controls")
        viz_control_layout = QVBoxLayout(viz_control_group)
        
        # Point size control
        size_layout = QHBoxLayout()
        size_layout.addWidget(QLabel("Point Size:"))
        self.point_size_slider = QSlider(Qt.Orientation.Horizontal)
        self.point_size_slider.setRange(1, 20)
        self.point_size_slider.setValue(3)
        self.point_size_slider.valueChanged.connect(self.update_point_size)
        size_layout.addWidget(self.point_size_slider)
        self.point_size_label = QLabel("3")
        size_layout.addWidget(self.point_size_label)
        viz_control_layout.addLayout(size_layout)
        
        # Downsampling control
        downsample_layout = QHBoxLayout()
        downsample_layout.addWidget(QLabel("Downsample:"))
        self.downsample_spin = QSpinBox()
        self.downsample_spin.setRange(1, 100)
        self.downsample_spin.setValue(1)
        self.downsample_spin.setSuffix(" (every Nth point)")
        self.downsample_spin.valueChanged.connect(self.update_visualization)
        downsample_layout.addWidget(self.downsample_spin)
        viz_control_layout.addLayout(downsample_layout)
        
        # Color options
        self.color_by_height = QCheckBox("Color by Height")
        self.color_by_height.stateChanged.connect(self.update_visualization)
        viz_control_layout.addWidget(self.color_by_height)
        
        # Reset view button
        self.reset_view_btn = QPushButton("Reset View")
        self.reset_view_btn.clicked.connect(self.reset_camera_view)
        viz_control_layout.addWidget(self.reset_view_btn)
        
        left_layout.addWidget(viz_control_group)
        
        # Stats
        stats_group = QGroupBox("PointCloud Statistics")
        stats_layout = QGridLayout(stats_group)
        
        self.stats_labels = {}
        stats = ["Points", "Width", "Height", "Fields", "Frame ID", "Last Update"]
        for i, stat in enumerate(stats):
            label = QLabel(f"{stat}:")
            label.setFont(QFont("Arial", 8))
            value = QLabel("N/A")
            value.setFont(QFont("Arial", 8, QFont.Weight.Bold))
            stats_layout.addWidget(label, i, 0)
            stats_layout.addWidget(value, i, 1)
            self.stats_labels[stat] = value
        
        left_layout.addWidget(stats_group)
        
        # Status display
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        self.display_text = QPlainTextEdit()
        self.display_text.setReadOnly(True)
        self.display_text.setMaximumHeight(120)
        self.display_text.setPlainText("No pointcloud data received yet.")
        status_layout.addWidget(self.display_text)
        left_layout.addWidget(status_group)
        
        left_layout.addStretch()
        main_layout.addWidget(left_panel)
        
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.VLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        main_layout.addWidget(separator)
        
        # Right panel for 3D visualization
        viz_group = QGroupBox("3D PointCloud Viewer")
        viz_layout = QVBoxLayout(viz_group)
        
        # Create OpenGL widget
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setMinimumSize(600, 400)
        
        # Set up the 3D scene
        self.setup_3d_scene()
        
        viz_layout.addWidget(self.gl_widget)
        main_layout.addWidget(viz_group)
        
        # Set stretch factors
        main_layout.setStretch(0, 0)  # Left panel fixed
        main_layout.setStretch(2, 1)  # Right panel stretches
    
    def setup_3d_scene(self):
        """Set up the 3D visualization scene"""
        # Set background color
        self.gl_widget.setBackgroundColor('k')  # Black background
        
        # Set initial camera position
        self.gl_widget.setCameraPosition(distance=10, elevation=30, azimuth=45)
        
        # Add coordinate axes
        self.add_coordinate_axes()
        
        # Add grid
        self.add_grid()
        
        # Initialize scatter plot item
        self.scatter = None
        
    def add_coordinate_axes(self):
        """Add X, Y, Z coordinate axes to the scene"""
        # X axis (red)
        x_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [2, 0, 0]]),
            color=(1, 0, 0, 1),
            width=3
        )
        self.gl_widget.addItem(x_axis)
        
        # Y axis (green)
        y_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 2, 0]]),
            color=(0, 1, 0, 1),
            width=3
        )
        self.gl_widget.addItem(y_axis)
        
        # Z axis (blue)
        z_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, 2]]),
            color=(0, 0, 1, 1),
            width=3
        )
        self.gl_widget.addItem(z_axis)
    
    def add_grid(self):
        """Add a grid to the scene"""
        grid = gl.GLGridItem()
        grid.setSize(20, 20)
        grid.setSpacing(1, 1)
        grid.translate(0, 0, 0)
        self.gl_widget.addItem(grid)
    
    def reset_camera_view(self):
        """Reset camera to default view"""
        self.gl_widget.setCameraPosition(distance=10, elevation=30, azimuth=45)
        self._camera_adjusted = False  # Allow auto-adjustment on next pointcloud
    
    def update_point_size(self):
        """Update point size and refresh visualization"""
        size = self.point_size_slider.value()
        self.point_size_label.setText(str(size))
        self.update_visualization()
    
    def update_available_topics(self, topics):
        """Update the available topics in combo box"""
        self.available_topics = topics
        
        current_text = self.topic_combo.currentText()
        self.topic_combo.clear()
        
        if topics:
            self.topic_combo.addItems(topics)
            self.topic_combo.setPlaceholderText("Select pointcloud topic...")
            self.connect_btn.setEnabled(True)
        else:
            self.topic_combo.setPlaceholderText("No pointcloud topics available")
            self.connect_btn.setEnabled(False)
        
        # Restore previous selection if it still exists
        if current_text and current_text in topics:
            self.topic_combo.setCurrentText(current_text)
    
    def connect_pointcloud(self):
        """Connect to selected pointcloud topic"""
        topic = self.topic_combo.currentText().strip()
        if topic and topic in self.available_topics:
            success = self.main_window.connect_generic_topic(topic, 'sensor_msgs/msg/PointCloud2', 'pointcloud')
            if success:
                self.connected_topic = topic
                self.connect_btn.setEnabled(False)
                self.disconnect_btn.setEnabled(True)
                self.display_text.setPlainText(f"Connected to {topic}. Waiting for data...")
                self._camera_adjusted = False  # Reset camera adjustment flag
            else:
                self.display_text.setPlainText(f"Failed to connect to {topic}")
    
    def disconnect_pointcloud(self):
            """Disconnect from current pointcloud topic"""
            if self.connected_topic:
                success = self.main_window.disconnect_generic_topic(self.connected_topic)
                if success:
                    self.connected_topic = None
                    self.connect_btn.setEnabled(True)
                    self.disconnect_btn.setEnabled(False)
                    self.display_text.setPlainText("Disconnected from pointcloud topic.")
                    self.clear_visualization()
                    self.update_stats()
                else:
                    self.display_text.setPlainText("Failed to disconnect from pointcloud topic")
        
    def clear_visualization(self):
        """Clear the current visualization"""
        if self.scatter:
            self.gl_widget.removeItem(self.scatter)
            self.scatter = None
        self.current_pointcloud = None
        self.raw_points = None
    
    def update_pointcloud(self, msg, topic_name):
        """Update pointcloud visualization with new data"""
        try:
            if topic_name != self.connected_topic:
                return
            
            # Parse the pointcloud message
            points = self.parse_pointcloud2(msg)
            if points is None or len(points) == 0:
                self.display_text.setPlainText("Received empty pointcloud")
                return
            
            self.current_pointcloud = msg
            self.raw_points = points
            
            # Update statistics
            self.update_stats()
            
            # Update visualization
            self.update_visualization()
            
            # Auto-adjust camera on first pointcloud
            if not self._camera_adjusted and len(points) > 0:
                self.auto_adjust_camera(points)
                self._camera_adjusted = True
            
            # Update status
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.display_text.setPlainText(
                f"PointCloud received at {timestamp}\n"
                f"Points: {len(points)}\n"
                f"Frame: {msg.header.frame_id}\n"
                f"Topic: {topic_name}"
            )
            
        except Exception as e:
            self.display_text.setPlainText(f"Error processing pointcloud: {str(e)}")
    
    def parse_pointcloud2(self, msg):
        """Parse PointCloud2 message and extract XYZ points"""
        try:
            # Use point_cloud2 utility to read points
            points_list = []
            
            # Get point step and point data
            point_step = msg.point_step
            row_step = msg.row_step
            data = msg.data
            
            # Find field offsets for x, y, z
            x_offset = y_offset = z_offset = None
            
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                return None
            
            # Extract points
            for i in range(0, len(data), point_step):
                if i + 12 <= len(data):  # Ensure we have enough bytes for x, y, z
                    x = np.frombuffer(data[i + x_offset:i + x_offset + 4], dtype=np.float32)[0]
                    y = np.frombuffer(data[i + y_offset:i + y_offset + 4], dtype=np.float32)[0]
                    z = np.frombuffer(data[i + z_offset:i + z_offset + 4], dtype=np.float32)[0]
                    
                    # Filter out invalid points
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                           np.isinf(x) or np.isinf(y) or np.isinf(z)):
                        points_list.append([x, y, z])
            
            return np.array(points_list) if points_list else None
            
        except Exception as e:
            print(f"Error parsing PointCloud2: {str(e)}")
            return None
    
    def auto_adjust_camera(self, points):
        """Auto-adjust camera position based on pointcloud bounds"""
        if len(points) == 0:
            return
        
        # Calculate bounding box
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        center = (min_vals + max_vals) / 2
        size = np.max(max_vals - min_vals)
        
        # Set camera distance based on pointcloud size
        distance = max(size * 1.5, 5.0)  # Minimum distance of 5
        
        # Set camera position
        self.gl_widget.setCameraPosition(
            distance=distance,
            elevation=30,
            azimuth=45,
            center=center
        )
    
    def update_visualization(self):
        """Update the 3D visualization with current settings"""
        if self.raw_points is None or len(self.raw_points) == 0:
            return
        
        try:
            # Apply downsampling
            downsample_factor = self.downsample_spin.value()
            if downsample_factor > 1:
                points = self.raw_points[::downsample_factor]
            else:
                points = self.raw_points
            
            if len(points) == 0:
                return
            
            # Prepare colors
            if self.color_by_height.isChecked():
                # Color by Z coordinate (height)
                z_values = points[:, 2]
                z_min, z_max = np.min(z_values), np.max(z_values)
                if z_max > z_min:
                    # Normalize Z values to 0-1
                    normalized_z = (z_values - z_min) / (z_max - z_min)
                    # Create color map (blue to red based on height)
                    colors = np.zeros((len(points), 4))
                    colors[:, 0] = normalized_z  # Red channel
                    colors[:, 1] = 1 - normalized_z  # Green channel  
                    colors[:, 2] = 1 - normalized_z  # Blue channel
                    colors[:, 3] = 1.0  # Alpha channel
                else:
                    # All points at same height, use white
                    colors = np.ones((len(points), 4))
            else:
                # Use white color for all points
                colors = np.ones((len(points), 4))
            
            # Remove existing scatter plot
            if self.scatter:
                self.gl_widget.removeItem(self.scatter)
            
            # Create new scatter plot
            point_size = self.point_size_slider.value()
            self.scatter = gl.GLScatterPlotItem(
                pos=points,
                color=colors,
                size=point_size,
                pxMode=True
            )
            
            # Add to scene
            self.gl_widget.addItem(self.scatter)
            
        except Exception as e:
            self.display_text.appendPlainText(f"Visualization error: {str(e)}")
    
    def update_stats(self):
        """Update statistics display"""
        if self.current_pointcloud is None:
            # Reset all stats
            for label in self.stats_labels.values():
                label.setText("N/A")
            return
        
        msg = self.current_pointcloud
        
        # Calculate number of points
        if self.raw_points is not None:
            num_points = len(self.raw_points)
        else:
            num_points = msg.width * msg.height if msg.width and msg.height else 0
        
        # Update labels
        self.stats_labels["Points"].setText(f"{num_points:,}")
        self.stats_labels["Width"].setText(str(msg.width))
        self.stats_labels["Height"].setText(str(msg.height))
        self.stats_labels["Fields"].setText(", ".join([f.name for f in msg.fields]))
        self.stats_labels["Frame ID"].setText(msg.header.frame_id)
        self.stats_labels["Last Update"].setText(datetime.now().strftime("%H:%M:%S"))
    
    def closeEvent(self, event):
        """Clean up when tab is closed"""
        if self.connected_topic:
            self.disconnect_pointcloud()
        event.accept()
        
