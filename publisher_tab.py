from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
                            QComboBox, QGroupBox, QGridLayout, QLineEdit, QSpinBox,
                            QDoubleSpinBox, QCheckBox, QTextEdit, QTabWidget, QScrollArea)
from PyQt6.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_msgs.msg import String, Bool, Float64, Int32, Float32, Int64, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from builtin_interfaces.msg import Time
import json
from datetime import datetime


class PublisherNode(Node):
    """Dedicated node for publishing messages"""
    def __init__(self):
        super().__init__('multi_tab_publisher')
        self._custom_publishers = {}  # ✅ doesn't clash with rclpy internals

    def create_topic_publisher(self, topic_name, msg_type, qos_profile=None):
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        msg_type_map = {
            'geometry_msgs/msg/Twist': Twist,
            'geometry_msgs/msg/PoseStamped': PoseStamped,
            'std_msgs/msg/String': String,
            'std_msgs/msg/Bool': Bool,
            'std_msgs/msg/Float64': Float64,
            'std_msgs/msg/Float32': Float32,
            'std_msgs/msg/Int32': Int32,
            'std_msgs/msg/Int64': Int64,
            'nav_msgs/msg/Odometry': Odometry,
        }

        if msg_type in msg_type_map:
            publisher = self.create_publisher(
                msg_type_map[msg_type],
                topic_name,
                qos_profile
            )
            self._custom_publishers[topic_name] = publisher  # ✅ safe name
            return publisher
        else:
            self.get_logger().warn(f'Unsupported message type for publishing: {msg_type}')
            return None

    def remove_publisher(self, topic_name):
        if topic_name in self._custom_publishers:
            del self._custom_publishers[topic_name]



class TwistPublisherWidget(QWidget):
    """Widget for publishing Twist messages (velocity commands)"""
    def __init__(self, publisher_node):
        super().__init__()
        self.publisher_node = publisher_node
        self.publisher = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_message)
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Topic configuration
        config_group = QGroupBox("Topic Configuration")
        config_layout = QGridLayout(config_group)
        
        config_layout.addWidget(QLabel("Topic Name:"), 0, 0)
        self.topic_edit = QLineEdit("/cmd_vel")
        config_layout.addWidget(self.topic_edit, 0, 1)
        
        config_layout.addWidget(QLabel("Publish Rate (Hz):"), 0, 2)
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 100)
        self.rate_spin.setValue(10)
        config_layout.addWidget(self.rate_spin, 0, 3)
        
        self.start_btn = QPushButton("Start Publishing")
        self.start_btn.clicked.connect(self.start_publishing)
        config_layout.addWidget(self.start_btn, 1, 0)
        
        self.stop_btn = QPushButton("Stop Publishing")
        self.stop_btn.clicked.connect(self.stop_publishing)
        self.stop_btn.setEnabled(False)
        config_layout.addWidget(self.stop_btn, 1, 1)
        
        layout.addWidget(config_group)
        
        # Twist controls
        twist_group = QGroupBox("Twist Values")
        twist_layout = QGridLayout(twist_group)
        
        # Linear velocity
        twist_layout.addWidget(QLabel("Linear X:"), 0, 0)
        self.linear_x = QDoubleSpinBox()
        self.linear_x.setRange(-10.0, 10.0)
        self.linear_x.setSingleStep(0.1)
        self.linear_x.setDecimals(2)
        twist_layout.addWidget(self.linear_x, 0, 1)
        
        twist_layout.addWidget(QLabel("Linear Y:"), 0, 2)
        self.linear_y = QDoubleSpinBox()
        self.linear_y.setRange(-10.0, 10.0)
        self.linear_y.setSingleStep(0.1)
        self.linear_y.setDecimals(2)
        twist_layout.addWidget(self.linear_y, 0, 3)
        
        twist_layout.addWidget(QLabel("Linear Z:"), 1, 0)
        self.linear_z = QDoubleSpinBox()
        self.linear_z.setRange(-10.0, 10.0)
        self.linear_z.setSingleStep(0.1)
        self.linear_z.setDecimals(2)
        twist_layout.addWidget(self.linear_z, 1, 1)
        
        # Angular velocity
        twist_layout.addWidget(QLabel("Angular X:"), 1, 2)
        self.angular_x = QDoubleSpinBox()
        self.angular_x.setRange(-10.0, 10.0)
        self.angular_x.setSingleStep(0.1)
        self.angular_x.setDecimals(2)
        twist_layout.addWidget(self.angular_x, 1, 3)
        
        twist_layout.addWidget(QLabel("Angular Y:"), 2, 0)
        self.angular_y = QDoubleSpinBox()
        self.angular_y.setRange(-10.0, 10.0)
        self.angular_y.setSingleStep(0.1)
        self.angular_y.setDecimals(2)
        twist_layout.addWidget(self.angular_y, 2, 1)
        
        twist_layout.addWidget(QLabel("Angular Z:"), 2, 2)
        self.angular_z = QDoubleSpinBox()
        self.angular_z.setRange(-10.0, 10.0)
        self.angular_z.setSingleStep(0.1)
        self.angular_z.setDecimals(2)
        twist_layout.addWidget(self.angular_z, 2, 3)
        
        layout.addWidget(twist_group)
        
        # Quick controls
        quick_group = QGroupBox("Quick Controls")
        quick_layout = QGridLayout(quick_group)
        
        # Movement buttons
        buttons = [
            ("Forward", self.move_forward, 0, 1),
            ("Backward", self.move_backward, 2, 1),
            ("Left", self.move_left, 1, 0),
            ("Right", self.move_right, 1, 2),
            ("Turn Left", self.turn_left, 3, 0),
            ("Turn Right", self.turn_right, 3, 2),
            ("Stop", self.stop_movement, 1, 1)
        ]
        
        for text, callback, row, col in buttons:
            btn = QPushButton(text)
            btn.clicked.connect(callback)
            quick_layout.addWidget(btn, row, col)
        
        layout.addWidget(quick_group)
    
    def start_publishing(self):
        topic_name = self.topic_edit.text().strip()
        if not topic_name:
            return
        self.publisher = self.publisher_node.create_topic_publisher(topic_name, 'geometry_msgs/msg/Twist')
        if self.publisher:
            rate = self.rate_spin.value()
            self.timer.start(1000 // rate)  # Convert Hz to ms
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
    
    def stop_publishing(self):
        self.timer.stop()
        if self.publisher:
            topic_name = self.topic_edit.text().strip()
            self.publisher_node.remove_publisher(topic_name)
            self.publisher = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
    
    def publish_message(self):
        if not self.publisher:
            return
        
        msg = Twist()
        msg.linear.x = self.linear_x.value()
        msg.linear.y = self.linear_y.value()
        msg.linear.z = self.linear_z.value()
        msg.angular.x = self.angular_x.value()
        msg.angular.y = self.angular_y.value()
        msg.angular.z = self.angular_z.value()
        
        self.publisher.publish(msg)
    
    def move_forward(self):
        self.linear_x.setValue(0.5)
        self.angular_z.setValue(0.0)
    
    def move_backward(self):
        self.linear_x.setValue(-0.5)
        self.angular_z.setValue(0.0)
    
    def move_left(self):
        self.linear_x.setValue(0.0)
        self.linear_y.setValue(0.5)
    
    def move_right(self):
        self.linear_x.setValue(0.0)
        self.linear_y.setValue(-0.5)
    
    def turn_left(self):
        self.linear_x.setValue(0.0)
        self.angular_z.setValue(0.5)
    
    def turn_right(self):
        self.linear_x.setValue(0.0)
        self.angular_z.setValue(-0.5)
    
    def stop_movement(self):
        self.linear_x.setValue(0.0)
        self.linear_y.setValue(0.0)
        self.linear_z.setValue(0.0)
        self.angular_x.setValue(0.0)
        self.angular_y.setValue(0.0)
        self.angular_z.setValue(0.0)


class GenericPublisherWidget(QWidget):
    """Widget for publishing various message types"""
    def __init__(self, publisher_node):
        super().__init__()
        self.publisher_node = publisher_node
        self.publisher = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_message)
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Topic configuration
        config_group = QGroupBox("Publisher Configuration")
        config_layout = QGridLayout(config_group)
        
        config_layout.addWidget(QLabel("Topic Name:"), 0, 0)
        self.topic_edit = QLineEdit("/my_topic")
        config_layout.addWidget(self.topic_edit, 0, 1)
        
        config_layout.addWidget(QLabel("Message Type:"), 0, 2)
        self.msg_type_combo = QComboBox()
        self.msg_type_combo.addItems([
            'std_msgs/msg/String',
            'std_msgs/msg/Bool',
            'std_msgs/msg/Float64',
            'std_msgs/msg/Float32',
            'std_msgs/msg/Int32',
            'std_msgs/msg/Int64',
            'geometry_msgs/msg/PoseStamped',
            'sensor_msgs/msg/Joy'
        ])
        self.msg_type_combo.currentTextChanged.connect(self.update_message_fields)
        config_layout.addWidget(self.msg_type_combo, 0, 3)
        
        config_layout.addWidget(QLabel("Publish Rate (Hz):"), 1, 0)
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 100)
        self.rate_spin.setValue(1)
        config_layout.addWidget(self.rate_spin, 1, 1)
        
        self.continuous_check = QCheckBox("Continuous Publishing")
        config_layout.addWidget(self.continuous_check, 1, 2)
        
        self.start_btn = QPushButton("Start Publishing")
        self.start_btn.clicked.connect(self.start_publishing)
        config_layout.addWidget(self.start_btn, 2, 0)
        
        self.stop_btn = QPushButton("Stop Publishing")
        self.stop_btn.clicked.connect(self.stop_publishing)
        self.stop_btn.setEnabled(False)
        config_layout.addWidget(self.stop_btn, 2, 1)
        
        self.publish_once_btn = QPushButton("Publish Once")
        self.publish_once_btn.clicked.connect(self.publish_once)
        config_layout.addWidget(self.publish_once_btn, 2, 2)
        
        layout.addWidget(config_group)
        
        # Message fields (dynamic based on message type)
        self.fields_group = QGroupBox("Message Fields")
        self.fields_layout = QVBoxLayout(self.fields_group)
        layout.addWidget(self.fields_group)
        
        # Initialize with String message
        self.update_message_fields()
    
    def update_message_fields(self):
        # Clear existing fields
        for i in reversed(range(self.fields_layout.count())):
            child = self.fields_layout.itemAt(i).widget()
            if child:
                child.setParent(None)
        
        msg_type = self.msg_type_combo.currentText()
        
        if msg_type == 'std_msgs/msg/String':
            self.create_string_fields()
        elif msg_type == 'std_msgs/msg/Bool':
            self.create_bool_fields()
        elif msg_type in ['std_msgs/msg/Float64', 'std_msgs/msg/Float32']:
            self.create_float_fields()
        elif msg_type in ['std_msgs/msg/Int32', 'std_msgs/msg/Int64']:
            self.create_int_fields()
        elif msg_type == 'geometry_msgs/msg/PoseStamped':
            self.create_pose_stamped_fields()
        elif msg_type == 'sensor_msgs/msg/Joy':
            self.create_joy_fields()
    
    def create_string_fields(self):
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Data:"))
        self.string_edit = QLineEdit("Hello ROS2!")
        layout.addWidget(self.string_edit)
        self.fields_layout.addLayout(layout)
    
    def create_bool_fields(self):
        self.bool_check = QCheckBox("Boolean Value")
        self.fields_layout.addWidget(self.bool_check)
    
    def create_float_fields(self):
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Value:"))
        self.float_spin = QDoubleSpinBox()
        self.float_spin.setRange(-1000000.0, 1000000.0)
        self.float_spin.setDecimals(6)
        layout.addWidget(self.float_spin)
        self.fields_layout.addLayout(layout)
    
    def create_int_fields(self):
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Value:"))
        self.int_spin = QSpinBox()
        self.int_spin.setRange(-2147483648, 2147483647)
        layout.addWidget(self.int_spin)
        self.fields_layout.addLayout(layout)
    
    def create_pose_stamped_fields(self):
        # Frame ID
        frame_layout = QHBoxLayout()
        frame_layout.addWidget(QLabel("Frame ID:"))
        self.frame_id_edit = QLineEdit("base_link")
        frame_layout.addWidget(self.frame_id_edit)
        self.fields_layout.addLayout(frame_layout)
        
        # Position
        pos_group = QGroupBox("Position")
        pos_layout = QGridLayout(pos_group)
        
        pos_layout.addWidget(QLabel("X:"), 0, 0)
        self.pos_x = QDoubleSpinBox()
        self.pos_x.setRange(-1000.0, 1000.0)
        self.pos_x.setDecimals(3)
        pos_layout.addWidget(self.pos_x, 0, 1)
        
        pos_layout.addWidget(QLabel("Y:"), 0, 2)
        self.pos_y = QDoubleSpinBox()
        self.pos_y.setRange(-1000.0, 1000.0)
        self.pos_y.setDecimals(3)
        pos_layout.addWidget(self.pos_y, 0, 3)
        
        pos_layout.addWidget(QLabel("Z:"), 1, 0)
        self.pos_z = QDoubleSpinBox()
        self.pos_z.setRange(-1000.0, 1000.0)
        self.pos_z.setDecimals(3)
        pos_layout.addWidget(self.pos_z, 1, 1)
        
        self.fields_layout.addWidget(pos_group)
        
        # Orientation
        orient_group = QGroupBox("Orientation (Quaternion)")
        orient_layout = QGridLayout(orient_group)
        
        orient_layout.addWidget(QLabel("X:"), 0, 0)
        self.orient_x = QDoubleSpinBox()
        self.orient_x.setRange(-1.0, 1.0)
        self.orient_x.setDecimals(4)
        orient_layout.addWidget(self.orient_x, 0, 1)
        
        orient_layout.addWidget(QLabel("Y:"), 0, 2)
        self.orient_y = QDoubleSpinBox()
        self.orient_y.setRange(-1.0, 1.0)
        self.orient_y.setDecimals(4)
        orient_layout.addWidget(self.orient_y, 0, 3)
        
        orient_layout.addWidget(QLabel("Z:"), 1, 0)
        self.orient_z = QDoubleSpinBox()
        self.orient_z.setRange(-1.0, 1.0)
        self.orient_z.setDecimals(4)
        orient_layout.addWidget(self.orient_z, 1, 1)
        
        orient_layout.addWidget(QLabel("W:"), 1, 2)
        self.orient_w = QDoubleSpinBox()
        self.orient_w.setRange(-1.0, 1.0)
        self.orient_w.setDecimals(4)
        self.orient_w.setValue(1.0)  # Default to valid quaternion
        orient_layout.addWidget(self.orient_w, 1, 3)
        
        self.fields_layout.addWidget(orient_group)
    
    def create_joy_fields(self):
        # Axes
        axes_layout = QHBoxLayout()
        axes_layout.addWidget(QLabel("Axes (comma-separated):"))
        self.axes_edit = QLineEdit("0.0, 0.0, 0.0, 0.0")
        axes_layout.addWidget(self.axes_edit)
        self.fields_layout.addLayout(axes_layout)
        
        # Buttons
        buttons_layout = QHBoxLayout()
        buttons_layout.addWidget(QLabel("Buttons (comma-separated 0/1):"))
        self.buttons_edit = QLineEdit("0, 0, 0, 0")
        buttons_layout.addWidget(self.buttons_edit)
        self.fields_layout.addLayout(buttons_layout)
    
    def start_publishing(self):
        topic_name = self.topic_edit.text().strip()
        if not topic_name:
            return
        
        msg_type = self.msg_type_combo.currentText()
        self.publisher = self.publisher_node.create_topic_publisher(topic_name, msg_type)
        
        if self.publisher and self.continuous_check.isChecked():
            rate = self.rate_spin.value()
            self.timer.start(1000 // rate)
            
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
    
    def stop_publishing(self):
        self.timer.stop()
        if self.publisher:
            topic_name = self.topic_edit.text().strip()
            self.publisher_node.remove_publisher(topic_name)
            self.publisher = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
    
    def publish_once(self):
        topic_name = self.topic_edit.text().strip()
        if not topic_name:
            return
        
        if not self.publisher:
            msg_type = self.msg_type_combo.currentText()
            self.publisher = self.publisher_node.create_topic_publisher(topic_name, msg_type)
        
        if self.publisher:
            self.publish_message()
    
    def publish_message(self):
        if not self.publisher:
            return
        
        msg_type = self.msg_type_combo.currentText()
        
        try:
            if msg_type == 'std_msgs/msg/String':
                msg = String()
                msg.data = self.string_edit.text()
            elif msg_type == 'std_msgs/msg/Bool':
                msg = Bool()
                msg.data = self.bool_check.isChecked()
            elif msg_type == 'std_msgs/msg/Float64':
                msg = Float64()
                msg.data = self.float_spin.value()
            elif msg_type == 'std_msgs/msg/Float32':
                msg = Float32()
                msg.data = float(self.float_spin.value())
            elif msg_type == 'std_msgs/msg/Int32':
                msg = Int32()
                msg.data = self.int_spin.value()
            elif msg_type == 'std_msgs/msg/Int64':
                msg = Int64()
                msg.data = self.int_spin.value()
            elif msg_type == 'geometry_msgs/msg/PoseStamped':
                msg = PoseStamped()
                msg.header.frame_id = self.frame_id_edit.text()
                msg.header.stamp = self.publisher_node.get_clock().now().to_msg()
                msg.pose.position.x = self.pos_x.value()
                msg.pose.position.y = self.pos_y.value()
                msg.pose.position.z = self.pos_z.value()
                msg.pose.orientation.x = self.orient_x.value()
                msg.pose.orientation.y = self.orient_y.value()
                msg.pose.orientation.z = self.orient_z.value()
                msg.pose.orientation.w = self.orient_w.value()
            elif msg_type == 'sensor_msgs/msg/Joy':
                msg = Joy()
                msg.header.stamp = self.publisher_node.get_clock().now().to_msg()
                # Parse axes
                axes_str = self.axes_edit.text().split(',')
                msg.axes = [float(x.strip()) for x in axes_str if x.strip()]
                # Parse buttons
                buttons_str = self.buttons_edit.text().split(',')
                msg.buttons = [int(x.strip()) for x in buttons_str if x.strip()]
            
            self.publisher.publish(msg)
            
        except Exception as e:
            print(f"Error publishing message: {str(e)}")


class PublisherTab(QWidget):
    """Main publisher tab containing all publisher widgets"""
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.publisher_node = None
        self.twist_widget = None
        self.generic_widget = None
        self.setup_ui()
        
        # Use a timer to delay initialization until ROS2 is ready
        self.init_timer = QTimer()
        self.init_timer.timeout.connect(self.check_ros_ready)
        self.init_timer.start(100)  # Check every 100ms
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Status
        status_group = QGroupBox("Publisher Status")
        status_layout = QHBoxLayout(status_group)
        self.status_label = QLabel("Waiting for ROS2 initialization...")
        status_layout.addWidget(self.status_label)
        layout.addWidget(status_group)
        
        # Tab widget for different publisher types
        self.publisher_tabs = QTabWidget()
        
        # Scroll areas for each tab
        twist_scroll = QScrollArea()
        twist_scroll.setWidgetResizable(True)
        twist_scroll.setWidget(QWidget())  # Placeholder
        
        generic_scroll = QScrollArea()
        generic_scroll.setWidgetResizable(True)
        generic_scroll.setWidget(QWidget())  # Placeholder
        
        self.publisher_tabs.addTab(twist_scroll, "🎮 Twist/Velocity")
        self.publisher_tabs.addTab(generic_scroll, "📝 Generic Messages")
        
        layout.addWidget(self.publisher_tabs)
    
    def check_ros_ready(self):
        """Check if ROS2 is ready and initialize publisher node"""
        if hasattr(self.main_window, 'node') and self.main_window.node is not None:
            self.init_timer.stop()
            self.init_publisher_node()
    
    def init_publisher_node(self):
        """Initialize the publisher node"""
        try:
            self.publisher_node = PublisherNode()
            self.status_label.setText("Publisher node ready")
            
            # Now create the actual widgets
            self.twist_widget = TwistPublisherWidget(self.publisher_node)
            self.generic_widget = GenericPublisherWidget(self.publisher_node)
            
            # Set them in the scroll areas
            twist_scroll = self.publisher_tabs.widget(0)
            twist_scroll.setWidget(self.twist_widget)
            
            generic_scroll = self.publisher_tabs.widget(1)
            generic_scroll.setWidget(self.generic_widget)
            
        except Exception as e:
            self.status_label.setText(f"Error initializing publisher: {str(e)}")
    
    def cleanup(self):
        """Clean up timers and publishers"""
        if self.init_timer.isActive():
            self.init_timer.stop()
            
        if self.twist_widget:
            self.twist_widget.stop_publishing()
        if self.generic_widget:
            self.generic_widget.stop_publishing()
        
        if self.publisher_node:
            self.publisher_node.destroy_node()