#!/usr/bin/env python3

import sys
import threading
import signal
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget,
    QTabWidget, QTextEdit, QGroupBox, QPushButton, QSplitter
)
from PyQt6.QtCore import QTimer, Qt
from ros_utils import (
    ImageSubscriber, CompressedImageSubscriber, determine_image_topic_type,
    MessageSignals, GenericSubscriber
)
from image_tab import ImageTab
from pointcloud_tab import PointCloudTab
from publisher_tab import PublisherTab
from monitor_tab import MonitorTab
from style import style


class MultiTabROS2Viewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Multi-Tab Viewer & Publisher")
        style()
        self.setup_fullscreen()

        self.monitor_image_subscriber = None
        self.monitor_subscriber_thread = None
        self.node = None
        self.ros_thread = None
        self.subscribers = {}
        self.image_subscribers = {}
        self.pointcloud_subscribers = {}
        self.signals = MessageSignals()

        self.setup_ui()
        self.init_ros2()

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)

    def setup_fullscreen(self):
        """Set up the window to be fullscreen but not larger than screen"""
        screen = QApplication.primaryScreen()
        if screen:
            screen_geometry = screen.availableGeometry()
            margin = 50
            width = screen_geometry.width() - margin
            height = screen_geometry.height() - margin
            x = screen_geometry.x() + margin // 2
            y = screen_geometry.y() + margin // 2
            self.setGeometry(x, y, width, height)
            self.setFixedSize(width, height)
        else:
            self.setGeometry(50, 50, 1500, 900)
            self.setFixedSize(1500, 900)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        central_layout = QVBoxLayout(central_widget)
        central_layout.setContentsMargins(15, 15, 15, 15)
        central_layout.setSpacing(10)
        central_layout.addWidget(main_splitter)

        # Left side
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 10, 0)
        left_layout.setSpacing(10)

        self.tab_widget = QTabWidget()
        self.image_tab = ImageTab(self.signals, self, 5)
        self.pointcloud_tab = PointCloudTab(self.signals, self)
        self.publisher_tab = PublisherTab(self)
        self.monitor_tab = MonitorTab(self.signals, self)

        self.tab_widget.addTab(self.image_tab, "Images")
        self.tab_widget.addTab(self.pointcloud_tab, "PointCloud")
        self.tab_widget.addTab(self.publisher_tab, "Publishers")
        self.tab_widget.addTab(self.monitor_tab, "Monitor")

        left_layout.addWidget(self.tab_widget)

        # Right side
        right_widget = QWidget()
        right_widget.setMinimumWidth(400)
        right_widget.setMaximumWidth(600)
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 0, 0, 0)
        right_layout.setSpacing(10)

        topics_group = QGroupBox("Available Topics")
        topics_layout = QVBoxLayout(topics_group)
        topics_layout.setContentsMargins(15, 20, 15, 15)
        topics_layout.setSpacing(12)

        refresh_btn = QPushButton("Refresh Topics")
        refresh_btn.setObjectName("refreshButton")
        refresh_btn.clicked.connect(self.refresh_topics)
        refresh_btn.setToolTip("Refresh the list of available ROS2 topics")
        topics_layout.addWidget(refresh_btn)

        self.topics_text = QTextEdit()
        self.topics_text.setReadOnly(True)
        self.topics_text.setMinimumHeight(400)
        self.topics_text.setToolTip("List of available ROS2 topics organized by message type")
        topics_layout.addWidget(self.topics_text)

        right_layout.addWidget(topics_group)

        # Splitter setup
        main_splitter.addWidget(left_widget)
        main_splitter.addWidget(right_widget)

        current_width = self.width()
        left_width = int(current_width * 0.75)
        right_width = int(current_width * 0.25)
        main_splitter.setSizes([left_width, right_width])
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 0)

        self.statusBar().showMessage("ROS2 Multi-Tab Viewer Ready")
        self.statusBar().setStyleSheet("QStatusBar::item { border: none; }")

    def init_ros2(self):
        def ros_init():
            rclpy.init()
            self.node = Node('multi_tab_viewer_main')

        self.ros_thread = threading.Thread(target=ros_init, daemon=True)
        self.ros_thread.start()
        self.ros_thread.join()
        self.refresh_topics()

    def spin_ros(self):
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=0.001)

        for subscriber in list(self.image_subscribers.values()) + \
                         list(self.pointcloud_subscribers.values()) + \
                         list(self.subscribers.values()):
            if subscriber:
                rclpy.spin_once(subscriber, timeout_sec=0.001)

    def refresh_topics(self):
        if not self.node:
            return
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
            topics_by_type = {}
            for topic_name, topic_types in topic_names_and_types:
                for topic_type in topic_types:
                    topics_by_type.setdefault(topic_type, []).append(topic_name)

            topics_text = "Available Topics by Type:\n\n"
            for topic_type in sorted(topics_by_type.keys()):
                topics_text += f"{topic_type}:\n"
                for topic in sorted(topics_by_type[topic_type]):
                    topics_text += f"   └─ {topic}\n"
                topics_text += "\n"

            self.topics_text.setText(topics_text)

            image_topics = topics_by_type.get('sensor_msgs/msg/Image', []) + \
                           topics_by_type.get('sensor_msgs/msg/CompressedImage', [])

            self.image_tab.update_available_topics(sorted(image_topics))
            self.pointcloud_tab.update_available_topics(topics_by_type.get('sensor_msgs/msg/PointCloud2', []))
            self.monitor_tab.update_available_topics(sorted(image_topics))

            total_topics = len(set(t for ts in topics_by_type.values() for t in ts))
            self.statusBar().showMessage(
                f"ROS2 Multi-Tab Viewer Ready • {total_topics} topics available • Last updated: {self.get_current_time()}"
            )

        except Exception as e:
            self.topics_text.setText(f"Error getting topics: {str(e)}")
            self.statusBar().showMessage("Error refreshing topics")

    def get_current_time(self):
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S")

    def connect_image_camera(self, camera_id):
        """Connect to selected image topic for specific camera"""
        return self.image_tab.connect_camera(camera_id, self.image_subscribers, self.on_image_received)

    def disconnect_image_camera(self, camera_id):
        """Disconnect from current topic for specific camera"""
        return self.image_tab.disconnect_camera(camera_id, self.image_subscribers)

    def on_image_received(self, cv_image, subscriber_id):
        """Callback for when image is received from ROS topic"""
        self.signals.image_received.emit(cv_image, subscriber_id)

    def connect_pointcloud_topic(self, topic_name):
        """Connect to a pointcloud topic using dedicated PointCloudSubscriber"""
        from ros_utils import PointCloudSubscriber
        
        try:
            # Clean up existing subscriber if exists
            if topic_name in self.pointcloud_subscribers:
                old_subscriber = self.pointcloud_subscribers[topic_name]
                if old_subscriber:
                    old_subscriber.destroy_node()
                del self.pointcloud_subscribers[topic_name]

            # Create sanitized subscriber ID by removing invalid characters
            sanitized_id = topic_name.replace('/', '_').replace('-', '_').lstrip('_')
            
            # Create dedicated pointcloud subscriber
            subscriber = PointCloudSubscriber(
                topic_name, 
                self.on_pointcloud_received,
                sanitized_id 
            )
            
            self.pointcloud_subscribers[topic_name] = subscriber
            return True
            
        except Exception as e:
            print(f"Error connecting to pointcloud topic {topic_name}: {str(e)}")
            return False
    
    def disconnect_pointcloud_topic(self, topic_name):
        """Disconnect from a pointcloud topic"""
        if topic_name in self.pointcloud_subscribers:
            subscriber = self.pointcloud_subscribers[topic_name]
            if subscriber:
                subscriber.destroy_node()
            del self.pointcloud_subscribers[topic_name]
            return True
        return False

    def connect_generic_topic(self, topic_name, msg_type, callback_type):
        
        try:
            # For pointcloud topics, use the dedicated subscriber
            if msg_type == 'sensor_msgs/msg/PointCloud2' and callback_type == 'pointcloud':
                return self.connect_pointcloud_topic(topic_name)
            
            # Clean up existing subscriber if exists
            if topic_name in self.subscribers:
                old_subscriber = self.subscribers[topic_name]
                if old_subscriber:
                    old_subscriber.destroy_node()
                del self.subscribers[topic_name]

            # Create subscriber with appropriate callback
            subscriber_id = len(self.subscribers)
            
            if callback_type == 'monitor':
                subscriber = GenericSubscriber(topic_name, msg_type, self.on_generic_received, subscriber_id)
            else:
                subscriber = GenericSubscriber(topic_name, msg_type, self.on_generic_received, subscriber_id)
            
            self.subscribers[topic_name] = subscriber
            return True
            
        except Exception as e:
            print(f"Error connecting to topic {topic_name}: {str(e)}")
            return False

    def disconnect_generic_topic(self, topic_name):
        """Disconnect from a generic ROS topic"""
        # Check if it's a pointcloud topic first
        if topic_name in self.pointcloud_subscribers:
            return self.disconnect_pointcloud_topic(topic_name)
            
        # Otherwise handle as generic subscriber
        if topic_name in self.subscribers:
            subscriber = self.subscribers[topic_name]
            if subscriber:
                subscriber.destroy_node()
            del self.subscribers[topic_name]
            return True
        return False

    def on_pointcloud_received(self, msg, topic_name):
        """Callback for pointcloud messages"""
        self.signals.pointcloud_received.emit(msg, topic_name)

    def on_generic_received(self, msg, topic_name, msg_type):
        """Callback for generic messages"""
        try:
            # Convert message to string representation
            if hasattr(msg, 'data'):
                msg_str = str(msg.data)
            elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                msg_str = f"linear: [{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}], angular: [{msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f}]"
            elif hasattr(msg, 'pose') and hasattr(msg, 'twist'):
                pos = msg.pose.pose.position
                msg_str = f"pos: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]"
            elif hasattr(msg, 'ranges'):
                ranges = msg.ranges
                msg_str = f"ranges: {len(ranges)} points, min: {min(ranges):.2f}, max: {max(ranges):.2f}"
            else:
                msg_str = str(msg)[:200]  # Truncate long messages
            
            self.signals.generic_received.emit(msg_str, topic_name, msg_type)
            
        except Exception as e:
            self.signals.generic_received.emit(f"Error parsing message: {str(e)}", topic_name, msg_type)

    def connect_monitor_image_camera(self, topic_name, is_compressed=None):
        """Connect monitor tab to an image topic - handles both regular and compressed images"""
        try:
            if hasattr(self, 'monitor_image_subscriber') and self.monitor_image_subscriber:
                self.monitor_image_subscriber.destroy_node()
                self.monitor_image_subscriber = None
            
            if hasattr(self, 'monitor_subscriber_thread') and self.monitor_subscriber_thread:
                self.monitor_subscriber_thread = None
            
            if is_compressed is None:
                topic_type = determine_image_topic_type(self.node, topic_name)
                is_compressed = (topic_type == 'compressed')
            
            def monitor_image_callback(cv_image, camera_id=999):
                self.signals.image_received.emit(cv_image, 999)
            
            if is_compressed:
                self.monitor_image_subscriber = CompressedImageSubscriber(
                    topic_name, 
                    monitor_image_callback,
                    999  
                )
                print(f"Created CompressedImageSubscriber for {topic_name}")
            else:
                self.monitor_image_subscriber = ImageSubscriber(
                    topic_name, 
                    monitor_image_callback,
                    999  
                )
                print(f"Created ImageSubscriber for {topic_name}")
            
            def spin_monitor():
                print(f"Starting monitor spinning thread for {topic_name}")
                while (hasattr(self, 'monitor_image_subscriber') and 
                       self.monitor_image_subscriber is not None):
                    try:
                        rclpy.spin_once(self.monitor_image_subscriber, timeout_sec=0.1)
                    except Exception as e:
                        print(f"Error in monitor spinning: {e}")
                        break
                print(f"Monitor spinning thread stopped for {topic_name}")
            
            self.monitor_subscriber_thread = threading.Thread(target=spin_monitor, daemon=True)
            self.monitor_subscriber_thread.start()
            
            print(f"Monitor connected to {topic_name} ({'compressed' if is_compressed else 'raw'} image)")
            return True
            
        except Exception as e:
            print(f"Error connecting monitor to topic {topic_name}: {str(e)}")
            return False

    def disconnect_monitor_image_camera(self):
        """Disconnect monitor tab from image topic"""
        try:
            if hasattr(self, 'monitor_image_subscriber') and self.monitor_image_subscriber:
                self.monitor_image_subscriber.destroy_node()
                self.monitor_image_subscriber = None
            
            if hasattr(self, 'monitor_subscriber_thread'):
                self.monitor_subscriber_thread = None
            
            print("Monitor disconnected from image stream")
            return True
        except Exception as e:
            print(f"Error disconnecting monitor image stream: {str(e)}")
            return False

    def cleanup_and_exit(self):
        print("\nShutting down ROS2 Multi-Tab Viewer...")
        if hasattr(self, 'publisher_tab'):
            self.publisher_tab.cleanup()
        if self.monitor_image_subscriber:
            self.monitor_image_subscriber.destroy_node()
        for s in list(self.image_subscribers.values()) + \
                 list(self.pointcloud_subscribers.values()) + \
                 list(self.subscribers.values()):
            if s:
                s.destroy_node()
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        QApplication.quit()

    def closeEvent(self, event):
        self.cleanup_and_exit()
        event.accept()


def signal_handler(signum, frame, viewer):
    print(f"\nReceived signal {signum}")
    viewer.cleanup_and_exit()


def main():
    app = QApplication(sys.argv)
    viewer = MultiTabROS2Viewer()
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, viewer))
    signal.signal(signal.SIGTERM, lambda s, f: signal_handler(s, f, viewer))
    viewer.show()

    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)

    try:
        sys.exit(app.exec())
    except (KeyboardInterrupt, SystemExit):
        viewer.cleanup_and_exit()


if __name__ == '__main__':
    main()