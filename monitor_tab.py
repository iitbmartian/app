from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QComboBox, QPushButton, QTextEdit, QSplitter, QSlider, QTabWidget, QSpinBox
)
from PyQt6.QtCore import Qt, QTimer
from datetime import datetime
from joystick import CustomJoystick
from image_stuff import CameraWidget
from ros_utils import ImageSubscriber
from sensor_msgs.msg import Image, CompressedImage
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class MonitorTab(QWidget):
    def __init__(self, parent_signals, main_window):
        super().__init__()
        self.signals = parent_signals
        self.main_window = main_window
        self.available_topics = []
        self.connected_topic = None
        self.camera_widget = CameraWidget(0)  # Use camera ID 0 for monitor
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.publish_timer = QTimer()
        self.current_twist = (0.0, 0.0)
        self.twist_publisher = None
        self.twist_stamped_publisher = None
        self.current_message_type = "Twist"  # Default to Twist
        self.setup_ui()
        
        # Connect to parent signals - this is the key part from image_tab.py
        self.signals.image_received.connect(self.update_camera_image)
        
        self.publish_timer.timeout.connect(self.publish_twist)

    def setup_ui(self):
        layout = QVBoxLayout(self)
        self.tab_widget = QTabWidget()

        self.multi_window_tab = QWidget()
        self.setup_multi_window_tab(self.multi_window_tab)
        self.tab_widget.addTab(self.multi_window_tab, "🖥️ Multi-Window")

        self.original_monitor_tab = QWidget()
        self.setup_original_monitor_tab(self.original_monitor_tab)
        self.tab_widget.addTab(self.original_monitor_tab, "📊 Data Monitor")

        layout.addWidget(self.tab_widget)

    def setup_multi_window_tab(self, target_widget):
        layout = QVBoxLayout(target_widget)
        main_splitter = QSplitter(Qt.Orientation.Horizontal)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        image_group = QGroupBox("📷 Image Stream")
        image_layout = QVBoxLayout(image_group)

        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("Topic:"))
        self.image_topic_combo = QComboBox()
        self.image_topic_combo.setEditable(True)
        self.image_topic_combo.setPlaceholderText("No image topics available")
        self.image_topic_combo.setMinimumWidth(200)
        self.image_topic_combo.setEnabled(True)
        self.image_topic_combo.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        topic_layout.addWidget(self.image_topic_combo)

        self.image_connect_btn = QPushButton("Connect")
        self.image_connect_btn.clicked.connect(self.connect_image_stream)
        topic_layout.addWidget(self.image_connect_btn)

        self.image_disconnect_btn = QPushButton("Disconnect")
        self.image_disconnect_btn.clicked.connect(self.disconnect_image_stream)
        self.image_disconnect_btn.setEnabled(False)
        topic_layout.addWidget(self.image_disconnect_btn)

        image_layout.addLayout(topic_layout)
        
        # Add image type indicator
        self.image_type_label = QLabel("No image stream")
        self.image_type_label.setStyleSheet("color: gray; font-size: 10px;")
        image_layout.addWidget(self.image_type_label)
        
        image_layout.addWidget(self.camera_widget)
        left_layout.addWidget(image_group)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        joystick_group = QGroupBox("🕹️ Robot Control")
        joystick_layout = QVBoxLayout(joystick_group)
        self.joystick = CustomJoystick()
        self.joystick.positionChanged.connect(self.on_joystick_moved)
        joystick_layout.addWidget(self.joystick)
        self.joystick.setFocus()

        params_layout = QHBoxLayout()
        params_layout.addWidget(QLabel("Linear Speed:"))
        self.linear_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.linear_speed_slider.setRange(0, 100)
        self.linear_speed_slider.setValue(50)
        self.linear_speed_slider.valueChanged.connect(self.update_linear_speed)
        params_layout.addWidget(self.linear_speed_slider)
        self.linear_speed_label = QLabel("0.5")
        params_layout.addWidget(self.linear_speed_label)

        params_layout.addWidget(QLabel("Angular Speed:"))
        self.angular_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.angular_speed_slider.setRange(0, 200)
        self.angular_speed_slider.setValue(100)
        self.angular_speed_slider.valueChanged.connect(self.update_angular_speed)
        params_layout.addWidget(self.angular_speed_slider)
        self.angular_speed_label = QLabel("1.0")
        params_layout.addWidget(self.angular_speed_label)

        joystick_layout.addLayout(params_layout)

        twist_layout = QHBoxLayout()
        twist_layout.addWidget(QLabel("Cmd Topic:"))
        self.twist_topic_combo = QComboBox()
        self.twist_topic_combo.setEditable(True)
        self.twist_topic_combo.setCurrentText("/cmd_vel")
        self.twist_topic_combo.currentTextChanged.connect(self.on_topic_changed)
        twist_layout.addWidget(self.twist_topic_combo)

        self.enable_control_btn = QPushButton("Enable Control")
        self.enable_control_btn.setCheckable(True)
        self.enable_control_btn.toggled.connect(self.toggle_control)
        twist_layout.addWidget(self.enable_control_btn)

        joystick_layout.addLayout(twist_layout)

        # Enhanced control type layout
        control_type_layout = QHBoxLayout()
        control_type_layout.addWidget(QLabel("Message Type:"))
        self.control_type_combo = QComboBox()
        self.control_type_combo.addItem("Twist", "Twist")
        self.control_type_combo.addItem("TwistStamped", "TwistStamped")
        self.control_type_combo.setCurrentIndex(0)  # Default to Twist
        self.control_type_combo.currentTextChanged.connect(self.on_message_type_changed)
        control_type_layout.addWidget(self.control_type_combo)

        # Add status label for message type
        self.message_type_status = QLabel("Ready to publish Twist messages")
        self.message_type_status.setStyleSheet("color: blue; font-size: 10px;")
        control_type_layout.addWidget(self.message_type_status)

        joystick_layout.addLayout(control_type_layout)
        right_layout.addWidget(joystick_group)

        main_splitter.addWidget(left_widget)
        main_splitter.addWidget(right_widget)
        main_splitter.setSizes([600, 600])

        layout.addWidget(main_splitter)

    def setup_original_monitor_tab(self, target_widget):
        layout = QVBoxLayout(target_widget)
        label = QLabel("Original Monitor functionality will go here.")
        layout.addWidget(label)

    def on_topic_changed(self, topic_name):
        """Handle topic name changes and update publishers"""
        self.cleanup_publishers()
        self.setup_publishers(topic_name)

    def on_message_type_changed(self, display_text):
        """Handle message type changes"""
        # Extract the actual message type from the combo box data
        current_index = self.control_type_combo.currentIndex()
        if current_index >= 0:
            self.current_message_type = self.control_type_combo.itemData(current_index)
        
        # Update status
        self.message_type_status.setText(f"Ready to publish {self.current_message_type} messages")
        
        # Recreate publishers with current topic
        current_topic = self.twist_topic_combo.currentText().strip()
        if current_topic:
            self.cleanup_publishers()
            self.setup_publishers(current_topic)

    def setup_publishers(self, topic_name):
        """Setup publishers for the specified topic"""
        if not topic_name or not hasattr(self.main_window, 'node'):
            return

        try:
            # Create publisher based on message type
            if self.current_message_type == "TwistStamped":
                self.twist_stamped_publisher = self.main_window.node.create_publisher(
                    TwistStamped, topic_name, 10
                )
                print(f"Created TwistStamped publisher for topic: {topic_name}")
            else:  # Default to Twist
                self.twist_publisher = self.main_window.node.create_publisher(
                    Twist, topic_name, 10
                )
                print(f"Created Twist publisher for topic: {topic_name}")
                
            self.message_type_status.setText(f"✅ {self.current_message_type} publisher ready")
            self.message_type_status.setStyleSheet("color: green; font-size: 10px;")
            
        except Exception as e:
            print(f"Error creating publisher for {topic_name}: {str(e)}")
            self.message_type_status.setText(f"❌ Publisher creation failed")
            self.message_type_status.setStyleSheet("color: red; font-size: 10px;")

    def cleanup_publishers(self):
        """Clean up existing publishers"""
        if self.twist_publisher:
            try:
                self.main_window.node.destroy_publisher(self.twist_publisher)
            except Exception as e:
                print(f"Error destroying Twist publisher: {str(e)}")
            self.twist_publisher = None

        if self.twist_stamped_publisher:
            try:
                self.main_window.node.destroy_publisher(self.twist_stamped_publisher)
            except Exception as e:
                print(f"Error destroying TwistStamped publisher: {str(e)}")
            self.twist_stamped_publisher = None

    def create_twist_message(self, linear, angular):
        """Create a Twist message"""
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular
        return twist

    def create_twist_stamped_message(self, linear, angular):
        """Create a TwistStamped message"""
        twist_stamped = TwistStamped()
        
        # Set header
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self.main_window.node.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"  # Default frame
        
        # Set twist data
        twist_stamped.twist.linear.x = linear
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = angular
        
        return twist_stamped

    def publish_twist(self):
        """Publish twist message based on current message type"""
        topic = self.twist_topic_combo.currentText().strip()
        if not topic:
            return

        linear, angular = self.current_twist
        
        try:
            if self.current_message_type == "TwistStamped":
                if self.twist_stamped_publisher:
                    message = self.create_twist_stamped_message(linear, angular)
                    self.twist_stamped_publisher.publish(message)
                    print(f"Published TwistStamped to {topic}: linear={linear:.2f}, angular={angular:.2f}")
                else:
                    print(f"TwistStamped publisher not available for topic: {topic}")
            else:  # Twist
                if self.twist_publisher:
                    message = self.create_twist_message(linear, angular)
                    self.twist_publisher.publish(message)
                    print(f"Published Twist to {topic}: linear={linear:.2f}, angular={angular:.2f}")
                else:
                    print(f"Twist publisher not available for topic: {topic}")
                    
        except Exception as e:
            print(f"Error publishing {self.current_message_type} message to {topic}: {str(e)}")
            self.message_type_status.setText(f"❌ Publishing error")
            self.message_type_status.setStyleSheet("color: red; font-size: 10px;")

    def update_available_topics(self, topics):
        """Update available topics in combo boxes - enhanced for both regular and compressed images"""
        self.available_topics = topics
        
        if hasattr(self, 'image_topic_combo'):
            # Filter for both regular and compressed image topics
            # This should match what's done in image_tab.py
            image_topics = []
            for topic in topics:
                topic_lower = topic.lower()
                # Check for common image topic patterns
                if (('image' in topic_lower or 'camera' in topic_lower) and
                    (topic.endswith('/image_raw') or 
                     topic.endswith('/compressed') or
                     topic.endswith('/image') or
                     '/image_raw/' in topic or
                     '/compressed/' in topic or
                     topic.endswith('/image_color') or
                     topic.endswith('/image_mono') or
                     topic.endswith('/image_rect') or
                     topic.endswith('/image_rect_color'))):
                    image_topics.append(topic)
            
            current_image_text = self.image_topic_combo.currentText()
            self.image_topic_combo.clear()
            
            if image_topics:
                # Sort topics with compressed topics clearly marked
                sorted_topics = sorted(image_topics, key=lambda x: (x.split('/')[-1], x))
                
                # Add topics with visual indicators
                for topic in sorted_topics:
                    if self.is_compressed_topic(topic):
                        display_name = f"📦 {topic} (compressed)"
                    else:
                        display_name = f"🖼️ {topic} (raw)"
                    
                    self.image_topic_combo.addItem(display_name, topic)  # Store actual topic as data
                
                self.image_topic_combo.setPlaceholderText("Select image topic...")
                self.image_connect_btn.setEnabled(True)
            else:
                self.image_topic_combo.setPlaceholderText("No image topics available")
                self.image_connect_btn.setEnabled(False)
            
            # Restore previous selection if it still exists
            if current_image_text:
                for i in range(self.image_topic_combo.count()):
                    if self.image_topic_combo.itemData(i) == current_image_text:
                        self.image_topic_combo.setCurrentIndex(i)
                        break

        # Update twist topic combo with common cmd_vel topics
        if hasattr(self, 'twist_topic_combo'):
            current_twist_text = self.twist_topic_combo.currentText()
            self.twist_topic_combo.clear()
            
            # Add common command velocity topics
            common_cmd_topics = ["/cmd_vel", "/cmd_vel_mux/input/teleop", "/mobile_base/commands/velocity"]
            for topic in common_cmd_topics:
                self.twist_topic_combo.addItem(topic)
            
            # Add any other topics that might be command velocity topics
            cmd_topics = [topic for topic in topics if 'cmd' in topic.lower() or 'vel' in topic.lower()]
            for topic in cmd_topics:
                if topic not in common_cmd_topics:
                    self.twist_topic_combo.addItem(topic)
            
            # Restore previous selection or default to /cmd_vel
            if current_twist_text:
                index = self.twist_topic_combo.findText(current_twist_text)
                if index >= 0:
                    self.twist_topic_combo.setCurrentIndex(index)
                else:
                    self.twist_topic_combo.setCurrentText(current_twist_text)
            else:
                self.twist_topic_combo.setCurrentText("/cmd_vel")

    def is_compressed_topic(self, topic_name):
        """Check if a topic is a compressed image topic"""
        return (topic_name.endswith('/compressed') or 
                '/compressed' in topic_name or
                'compressed' in topic_name.lower())

    def get_selected_topic(self):
        """Get the actual topic name from the combo box selection"""
        current_index = self.image_topic_combo.currentIndex()
        if current_index >= 0:
            return self.image_topic_combo.itemData(current_index)
        else:
            # Fallback to text parsing if itemData is not available
            text = self.image_topic_combo.currentText().strip()
            if text.startswith('📦 ') or text.startswith('🖼️ '):
                # Extract topic name from display format
                parts = text.split(' ', 1)
                if len(parts) > 1:
                    topic_part = parts[1]
                    if topic_part.endswith(' (compressed)'):
                        return topic_part.replace(' (compressed)', '')
                    elif topic_part.endswith(' (raw)'):
                        return topic_part.replace(' (raw)', '')
                    else:
                        return topic_part
            return text

    def update_camera_image(self, cv_image, camera_id):
        """Update camera image - monitor uses camera ID 999"""
        if camera_id == 999:  # Monitor tab uses camera ID 999
            self.camera_widget.update_image(cv_image)
            
            # Update the image type indicator
            if self.connected_topic:
                is_compressed = self.is_compressed_topic(self.connected_topic)
                image_type = "Compressed" if is_compressed else "Raw"
                self.image_type_label.setText(f"📡 {image_type} Image Stream: {self.connected_topic}")
                self.image_type_label.setStyleSheet("color: green; font-size: 10px;")
            else:
                self.image_type_label.setText("No image stream")
                self.image_type_label.setStyleSheet("color: gray; font-size: 10px;")

    def connect_image_stream(self):
        """Connect to image stream using main window's method"""
        topic_name = self.get_selected_topic()
        
        if not topic_name or topic_name not in self.available_topics:
            print(f"Invalid topic: {topic_name}")
            return False

        try:
            # Determine if this is a compressed image topic
            is_compressed = self.is_compressed_topic(topic_name)
            
            print(f"Attempting to connect monitor to {topic_name} ({'compressed' if is_compressed else 'raw'} image)")
            
            success = self.main_window.connect_monitor_image_camera(topic_name, is_compressed)
            if success:
                self.camera_widget.set_topic(topic_name)
                self.image_connect_btn.setEnabled(False)
                self.image_disconnect_btn.setEnabled(True)
                self.connected_topic = topic_name
                
                # Update status immediately
                image_type = "Compressed" if is_compressed else "Raw"
                self.image_type_label.setText(f"📡 {image_type} Image Stream: {topic_name}")
                self.image_type_label.setStyleSheet("color: green; font-size: 10px;")
                
                print(f"Successfully connected monitor to {topic_name} ({'compressed' if is_compressed else 'raw'})")
                return True
            else:
                print(f"Failed to connect monitor to {topic_name}")
                self.image_type_label.setText("❌ Connection failed")
                self.image_type_label.setStyleSheet("color: red; font-size: 10px;")
                return False
        except Exception as e:
            print(f"Error connecting monitor to topic {topic_name}: {str(e)}")
            self.image_type_label.setText("❌ Connection error")
            self.image_type_label.setStyleSheet("color: red; font-size: 10px;")
            return False

    def disconnect_image_stream(self):
        """Disconnect from image stream"""
        try:
            success = self.main_window.disconnect_monitor_image_camera()
            if success:
                self.camera_widget.set_topic(None)
                self.image_connect_btn.setEnabled(True)
                self.image_disconnect_btn.setEnabled(False)
                self.connected_topic = None
                self.image_type_label.setText("No image stream")
                self.image_type_label.setStyleSheet("color: gray; font-size: 10px;")
                print("Successfully disconnected monitor image stream")
                return True
            else:
                print("Failed to disconnect monitor image stream")
                return False
        except Exception as e:
            print(f"Error disconnecting monitor image stream: {str(e)}")
            return False

    def on_joystick_moved(self, x, y):
        if self.enable_control_btn.isChecked():
            max_speed = 3.0
            norm_x = max(-1.0, min(1.0, x / max_speed))
            norm_y = max(-1.0, min(1.0, y / max_speed))
            linear = norm_y * self.linear_speed
            angular = -norm_x * self.angular_speed
            self.current_twist = (linear, angular)

    def update_linear_speed(self, value):
        self.linear_speed = value / 100.0
        self.linear_speed_label.setText(f"{self.linear_speed:.1f}")

    def update_angular_speed(self, value):
        self.angular_speed = value / 100.0
        self.angular_speed_label.setText(f"{self.angular_speed:.1f}")

    def toggle_control(self, enabled):
        if enabled:
            self.enable_control_btn.setText("Disable Control")
            self.enable_control_btn.setStyleSheet("background-color: #ffcccc;")
            
            # Setup publishers when control is enabled
            topic = self.twist_topic_combo.currentText().strip()
            if topic:
                self.setup_publishers(topic)
            
            self.publish_timer.start(100)  # Publish at 10 Hz
        else:
            self.enable_control_btn.setText("Enable Control")
            self.enable_control_btn.setStyleSheet("")
            self.publish_timer.stop()
            
            # Send stop command before disabling
            self.current_twist = (0.0, 0.0)
            self.publish_twist()
            
            # Clean up publishers
            self.cleanup_publishers()

    def closeEvent(self, event):
        """Clean up when the widget is closed"""
        self.cleanup_publishers()
        super().closeEvent(event)