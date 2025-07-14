from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QComboBox, QPushButton, QTextEdit, QSplitter, QSlider, QTabWidget, QSpinBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QScrollArea, QTreeWidget, QTreeWidgetItem
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont
from datetime import datetime
from joystick import CustomJoystick
from image_stuff import CameraWidget
from ros_utils import ImageSubscriber
from sensor_msgs.msg import Image, CompressedImage
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header, String, Int32, Int64, Float32, Float64, Bool
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray


class DataMonitorSubscriber:
    """Generic subscriber for monitoring different message types"""
    
    def __init__(self, node, topic_name, msg_type, callback):
        self.node = node
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.callback = callback
        self.subscriber = None
        self.message_count = 0
        self.last_message_time = None
        # Store reference to cleanup on close
        self.destroyed = False
        self.create_subscriber()
    
    def create_subscriber(self):
        """Create the subscriber based on message type"""
        try:
            if self.msg_type == "std_msgs/String":
                from std_msgs.msg import String
                self.subscriber = self.node.create_subscription(
                    String, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "std_msgs/Int32":
                from std_msgs.msg import Int32
                self.subscriber = self.node.create_subscription(
                    Int32, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "std_msgs/Int64":
                from std_msgs.msg import Int64
                self.subscriber = self.node.create_subscription(
                    Int64, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "std_msgs/Float32":
                from std_msgs.msg import Float32
                self.subscriber = self.node.create_subscription(
                    Float32, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "std_msgs/Float64":
                from std_msgs.msg import Float64
                self.subscriber = self.node.create_subscription(
                    Float64, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "std_msgs/Bool":
                from std_msgs.msg import Bool
                self.subscriber = self.node.create_subscription(
                    Bool, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "geometry_msgs/Twist":
                from geometry_msgs.msg import Twist
                self.subscriber = self.node.create_subscription(
                    Twist, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "geometry_msgs/TwistStamped":
                from geometry_msgs.msg import TwistStamped
                self.subscriber = self.node.create_subscription(
                    TwistStamped, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "sensor_msgs/BatteryState":
                from sensor_msgs.msg import BatteryState
                self.subscriber = self.node.create_subscription(
                    BatteryState, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "diagnostic_msgs/DiagnosticArray":
                from diagnostic_msgs.msg import DiagnosticArray
                self.subscriber = self.node.create_subscription(
                    DiagnosticArray, self.topic_name, self.message_callback, 10
                )
            elif self.msg_type == "husarion_ugv_msgs/ChargingStatus":
                try:
                    from husarion_ugv_msgs.msg import ChargingStatus
                    self.subscriber = self.node.create_subscription(
                        ChargingStatus, self.topic_name, self.message_callback, 10
                    )
                except ImportError:
                    print(f"Warning: husarion_ugv_msgs not available, cannot subscribe to {self.topic_name}")
                    return False
            else:
                print(f"Unsupported message type: {self.msg_type}")
                return False
            
            if self.subscriber is not None:
                print(f"Created subscriber for {self.topic_name} ({self.msg_type})")
                return True
            else:
                print(f"Failed to create subscriber for {self.topic_name}")
                return False
                
        except Exception as e:
            print(f"Error creating subscriber for {self.topic_name}: {str(e)}")
            # Set subscriber to None to indicate failure
            self.subscriber = None
            return False

    
    def message_callback(self, msg):
        """Handle incoming messages"""
        self.message_count += 1
        self.last_message_time = datetime.now()
        self.callback(self.topic_name, msg, self.message_count, self.last_message_time)
    
    def destroy(self):
        """Clean up the subscriber"""
        if self.subscriber:
            try:
                self.subscriber.destroy()  # Change from self.node.destroy_subscription()
                print(f"Destroyed subscriber for {self.topic_name}")
            except Exception as e:
                print(f"Error destroying subscriber for {self.topic_name}: {str(e)}")
            self.subscriber = None

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
        
        # Data monitor specific attributes
        self.data_subscribers = {}  # topic_name -> DataMonitorSubscriber
        self.topic_data = {}  # topic_name -> latest message data
        self.topic_stats = {}  # topic_name -> statistics
        
        self.setup_ui()
        
        # Connect to parent signals - this is the key part from image_tab.py
        self.signals.image_received.connect(self.update_camera_image)
        
        self.publish_timer.timeout.connect(self.publish_twist)

    def setup_ui(self):
        layout = QVBoxLayout(self)
        self.tab_widget = QTabWidget()

        self.multi_window_tab = QWidget()
        self.setup_multi_window_tab(self.multi_window_tab)
        self.tab_widget.addTab(self.multi_window_tab, "Multi-Window")

        self.original_monitor_tab = QWidget()
        self.setup_data_monitor_tab(self.original_monitor_tab)
        self.tab_widget.addTab(self.original_monitor_tab, "Data Monitor")

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

    def setup_data_monitor_tab(self, target_widget):
        """Setup the data monitor tab"""
        layout = QVBoxLayout(target_widget)
        
        # Control panel
        control_group = QGroupBox("🔧 Monitor Controls")
        control_layout = QVBoxLayout(control_group)
        
        # Topic selection
        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("Topic:"))
        self.monitor_topic_combo = QComboBox()
        self.monitor_topic_combo.setEditable(True)
        self.monitor_topic_combo.setPlaceholderText("Select or enter topic name")
        self.monitor_topic_combo.setMinimumWidth(200)
        # ADD THIS LINE FOR AUTO-COMPLETION:
        self.monitor_topic_combo.currentTextChanged.connect(self.auto_detect_message_type)
        topic_layout.addWidget(self.monitor_topic_combo)
        
        # Message type selection
        topic_layout.addWidget(QLabel("Message Type:"))
        self.monitor_msg_type_combo = QComboBox()
        self.monitor_msg_type_combo.addItems([
            "std_msgs/String",
            "std_msgs/Int32",
            "std_msgs/Int64", 
            "std_msgs/Float32",
            "std_msgs/Float64",
            "std_msgs/Bool",
            "geometry_msgs/Twist",
            "geometry_msgs/TwistStamped",
            "sensor_msgs/BatteryState",
            "diagnostic_msgs/DiagnosticArray",
            "husarion_ugv_msgs/ChargingStatus"
        ])
        topic_layout.addWidget(self.monitor_msg_type_combo)
        
        # Control buttons
        self.monitor_connect_btn = QPushButton("Start Monitoring")
        self.monitor_connect_btn.clicked.connect(self.start_topic_monitoring)
        topic_layout.addWidget(self.monitor_connect_btn)
        
        self.monitor_disconnect_btn = QPushButton("Stop Monitoring")
        self.monitor_disconnect_btn.clicked.connect(self.stop_topic_monitoring)
        self.monitor_disconnect_btn.setEnabled(False)
        topic_layout.addWidget(self.monitor_disconnect_btn)
        
        self.clear_data_btn = QPushButton("Clear Data")
        self.clear_data_btn.clicked.connect(self.clear_monitor_data)
        topic_layout.addWidget(self.clear_data_btn)
        
        control_layout.addLayout(topic_layout)
        layout.addWidget(control_group)
        
        # Status and statistics
        stats_group = QGroupBox("📈 Statistics")
        stats_layout = QVBoxLayout(stats_group)
        
        self.stats_label = QLabel("No active monitoring")
        self.stats_label.setStyleSheet("color: gray; font-size: 11px;")
        stats_layout.addWidget(self.stats_label)
        
        layout.addWidget(stats_group)
        
        # Data display area
        data_group = QGroupBox("Message Data")
        data_layout = QVBoxLayout(data_group)
        
        # Create splitter for different views
        data_splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Active topics list
        topics_widget = QWidget()
        topics_layout = QVBoxLayout(topics_widget)
        topics_layout.addWidget(QLabel("Active Topics:"))
        
        self.active_topics_list = QTreeWidget()
        self.active_topics_list.setHeaderLabels(["Topic", "Messages", "Last Update"])
        self.active_topics_list.itemClicked.connect(self.on_topic_selected)
        topics_layout.addWidget(self.active_topics_list)
        
        # Message details
        details_widget = QWidget()
        details_layout = QVBoxLayout(details_widget)
        details_layout.addWidget(QLabel("Message Details:"))
        
        self.message_details = QTextEdit()
        self.message_details.setReadOnly(True)
        self.message_details.setFont(QFont("Consolas", 10))
        details_layout.addWidget(self.message_details)
        
        data_splitter.addWidget(topics_widget)
        data_splitter.addWidget(details_widget)
        data_splitter.setSizes([300, 500])
        
        data_layout.addWidget(data_splitter)
        layout.addWidget(data_group)
        
        # Update timer for UI refresh
        self.monitor_update_timer = QTimer()
        self.monitor_update_timer.timeout.connect(self.update_monitor_display)
        self.monitor_update_timer.start(1000)  # Update every second

    def auto_detect_message_type(self, topic_name):
        """Auto-detect message type based on topic name patterns"""
        if not topic_name.strip():
            return
        
        # Common topic patterns and their likely message types
        patterns = {
            'battery': 'sensor_msgs/BatteryState',
            'diagnostics': 'diagnostic_msgs/DiagnosticArray',
            'charging': 'husarion_ugv_msgs/ChargingStatus',
            'cmd_vel': 'geometry_msgs/Twist',
            'velocity': 'geometry_msgs/Twist',
            'twist': 'geometry_msgs/Twist',
            'odom': 'nav_msgs/Odometry',
            'status': 'std_msgs/String',
            'state': 'std_msgs/String',
            'temperature': 'std_msgs/Float32',
            'pressure': 'std_msgs/Float32',
            'count': 'std_msgs/Int32',
            'enable': 'std_msgs/Bool',
            'active': 'std_msgs/Bool'
        }
        
        topic_lower = topic_name.lower()
        
        # Check patterns
        for pattern, msg_type in patterns.items():
            if pattern in topic_lower:
                # Find the message type in the combo box
                index = self.monitor_msg_type_combo.findText(msg_type)
                if index >= 0:
                    self.monitor_msg_type_combo.setCurrentIndex(index)
                    print(f"Auto-detected message type '{msg_type}' for topic '{topic_name}'")
                    return
        
        # Default based on topic structure
        if topic_name.endswith('/compressed'):
            # Compressed image topic
            return
        elif '/image' in topic_lower:
            # Image topic
            return
        elif topic_name.count('/') == 1:
            # Simple topic, likely std_msgs
            self.monitor_msg_type_combo.setCurrentText("std_msgs/String")


    def start_topic_monitoring(self):
        """Start monitoring a topic"""
        topic_name = self.get_selected_monitor_topic()
        msg_type = self.monitor_msg_type_combo.currentText()
        
        if not topic_name:
            self.stats_label.setText("Please enter a topic name")
            self.stats_label.setStyleSheet("color: red; font-size: 11px;")
            return
        
        # Validate topic name format
        if not topic_name.startswith('/'):
            topic_name = f"/{topic_name}"
        
        if topic_name in self.data_subscribers:
            self.stats_label.setText(f"Already monitoring {topic_name}")
            self.stats_label.setStyleSheet("color: orange; font-size: 11px;")
            return
        
        try:
            # Create subscriber
            subscriber = DataMonitorSubscriber(
                self.main_window.node,
                topic_name,
                msg_type,
                self.on_monitor_message_received
            )
            
            # Check if subscriber was created successfully
            if subscriber.subscriber is not None:
                self.data_subscribers[topic_name] = subscriber
                self.topic_data[topic_name] = {
                    'msg_type': msg_type,
                    'latest_message': None,
                    'message_history': []
                }
                self.topic_stats[topic_name] = {
                    'message_count': 0,
                    'start_time': datetime.now(),
                    'last_message_time': None,
                    'frequency': 0.0
                }
                
                self.update_active_topics_list()
                self.monitor_disconnect_btn.setEnabled(True)
                
                self.stats_label.setText(f"Monitoring {topic_name} ({msg_type})")
                self.stats_label.setStyleSheet("color: green; font-size: 11px;")
                
                print(f"Started monitoring {topic_name} ({msg_type})")
            else:
                self.stats_label.setText(f"Failed to create subscriber for {topic_name}")
                self.stats_label.setStyleSheet("color: red; font-size: 11px;")
                
        except Exception as e:
            self.stats_label.setText(f"Error: {str(e)}")
            self.stats_label.setStyleSheet("color: red; font-size: 11px;")
            print(f"Error starting monitoring for {topic_name}: {str(e)}")


    def stop_topic_monitoring(self):
        """Stop monitoring a specific topic or all topics"""
        topic_name = self.monitor_topic_combo.currentText().strip()
        
        if topic_name and topic_name in self.data_subscribers:
            # Stop monitoring specific topic
            self.data_subscribers[topic_name].destroy()
            del self.data_subscribers[topic_name]
            
            if topic_name in self.topic_data:
                del self.topic_data[topic_name]
            if topic_name in self.topic_stats:
                del self.topic_stats[topic_name]
            
            self.update_active_topics_list()
            self.stats_label.setText(f"Stopped monitoring {topic_name}")
            self.stats_label.setStyleSheet("color: blue; font-size: 11px;")
            
            print(f"Stopped monitoring {topic_name}")
        else:
            # Stop all monitoring
            for topic, subscriber in self.data_subscribers.items():
                subscriber.destroy()
            
            self.data_subscribers.clear()
            self.topic_data.clear()
            self.topic_stats.clear()
            
            self.update_active_topics_list()
            self.message_details.clear()
            self.stats_label.setText("Stopped all monitoring")
            self.stats_label.setStyleSheet("color: blue; font-size: 11px;")
            
            print("Stopped all topic monitoring")
        
        if not self.data_subscribers:
            self.monitor_disconnect_btn.setEnabled(False)

    def clear_monitor_data(self):
        """Clear all message data but keep subscriptions active"""
        for topic_name in self.topic_data:
            self.topic_data[topic_name]['message_history'].clear()
            self.topic_stats[topic_name]['message_count'] = 0
            self.topic_stats[topic_name]['start_time'] = datetime.now()
        
        self.message_details.clear()
        self.update_active_topics_list()
        self.stats_label.setText("Data cleared")
        self.stats_label.setStyleSheet("color: blue; font-size: 11px;")

    def on_monitor_message_received(self, topic_name, msg, message_count, timestamp):
        """Handle received messages for data monitoring"""
        print(f"DEBUG: Received message for {topic_name}, count: {message_count}")
        
        if topic_name not in self.topic_data:
            print(f"DEBUG: Topic {topic_name} not in topic_data")
            return
        
        # Update topic data
        self.topic_data[topic_name]['latest_message'] = msg
        self.topic_data[topic_name]['message_history'].append({
            'timestamp': timestamp,
            'message': msg,
            'count': message_count
        })
        
        # Limit history to last 100 messages
        if len(self.topic_data[topic_name]['message_history']) > 100:
            self.topic_data[topic_name]['message_history'] = self.topic_data[topic_name]['message_history'][-100:]
        
        # Update statistics
        self.topic_stats[topic_name]['message_count'] = message_count
        self.topic_stats[topic_name]['last_message_time'] = timestamp
        
        # Calculate frequency
        start_time = self.topic_stats[topic_name]['start_time']
        elapsed_time = (timestamp - start_time).total_seconds()
        if elapsed_time > 0:
            self.topic_stats[topic_name]['frequency'] = message_count / elapsed_time
        
        print(f"DEBUG: Updated data for {topic_name}, total messages: {message_count}")


    def update_monitor_display(self):
        """Update the monitor display with current data"""
        self.update_active_topics_list()
        
        if self.data_subscribers:
            # Update stats label with summary
            total_messages = sum(stats['message_count'] for stats in self.topic_stats.values())
            active_topics = len(self.data_subscribers)
            
            self.stats_label.setText(f"{active_topics} active topics, {total_messages} total messages")
            self.stats_label.setStyleSheet("color: green; font-size: 11px;")

    def update_active_topics_list(self):
        """Update the active topics list widget"""
        current_selection = None
        selected_items = self.active_topics_list.selectedItems()
        if selected_items:
            current_selection = selected_items[0].data(0, Qt.ItemDataRole.UserRole)
        
        self.active_topics_list.clear()
        
        for topic_name, stats in self.topic_stats.items():
            item = QTreeWidgetItem()
            item.setText(0, topic_name)
            item.setText(1, str(stats['message_count']))
            
            if stats['last_message_time']:
                last_update = stats['last_message_time'].strftime("%H:%M:%S")
                freq_str = f"{stats['frequency']:.1f} Hz"
                item.setText(2, f"{last_update} ({freq_str})")
            else:
                item.setText(2, "No messages")
            
            # Store topic name for selection
            item.setData(0, Qt.ItemDataRole.UserRole, topic_name)
            self.active_topics_list.addTopLevelItem(item)
            
            # Restore selection if this was the previously selected topic
            if current_selection == topic_name:
                self.active_topics_list.setCurrentItem(item)
                self.display_topic_messages(topic_name)
        
        # Auto-resize columns
        self.active_topics_list.resizeColumnToContents(0)
        self.active_topics_list.resizeColumnToContents(1)
        self.active_topics_list.resizeColumnToContents(2)
        
        # Auto-select first item if nothing is selected
        if not current_selection and self.active_topics_list.topLevelItemCount() > 0:
            first_item = self.active_topics_list.topLevelItem(0)
            self.active_topics_list.setCurrentItem(first_item)
            topic_name = first_item.data(0, Qt.ItemDataRole.UserRole)
            self.display_topic_messages(topic_name)

    def on_topic_selected(self, item):
        """Handle topic selection in the active topics list"""
        topic_name = item.data(0, Qt.ItemDataRole.UserRole)
        
        if topic_name in self.topic_data:
            self.display_topic_messages(topic_name)

    def display_topic_messages(self, topic_name):
        """Display messages for a specific topic"""
        if topic_name not in self.topic_data:
            return
        
        topic_info = self.topic_data[topic_name]
        msg_type = topic_info['msg_type']
        latest_msg = topic_info['latest_message']
        history = topic_info['message_history']
        
        # Clear and update display
        self.message_details.clear()
        
        # Header
        self.message_details.append(f"=== {topic_name} ({msg_type}) ===\n")
        
        # Latest message
        if latest_msg:
            self.message_details.append("Latest Message:")
            self.message_details.append(self.format_message(latest_msg))
            self.message_details.append("")
        
        # Recent messages
        if history:
            self.message_details.append("Recent Messages:")
            # Show last 10 messages
            for entry in history[-10:]:
                timestamp_str = entry['timestamp'].strftime("%H:%M:%S.%f")[:-3]
                self.message_details.append(f"[{timestamp_str}] Message #{entry['count']}:")
                self.message_details.append(self.format_message(entry['message']))
                self.message_details.append("")

    def format_message(self, msg):
        """Format a ROS message for display"""
        try:
            # Get the message type name
            msg_type = type(msg).__name__
            
            # Handle based on message type
            if msg_type == 'String':
                return f"  data: {msg.data}"
            elif msg_type in ['Int32', 'Int64', 'Float32', 'Float64', 'Bool']:
                return f"  data: {msg.data}"
            elif msg_type == 'Twist':
                return f"  linear:  x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}\n" \
                    f"  angular: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}"
            elif msg_type == 'TwistStamped':
                return f"  header: frame_id='{msg.header.frame_id}', stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n" \
                    f"  twist:\n" \
                    f"    linear:  x={msg.twist.linear.x:.3f}, y={msg.twist.linear.y:.3f}, z={msg.twist.linear.z:.3f}\n" \
                    f"    angular: x={msg.twist.angular.x:.3f}, y={msg.twist.angular.y:.3f}, z={msg.twist.angular.z:.3f}"
            elif msg_type == 'BatteryState':
                return self.format_battery_state(msg)
            elif msg_type == 'DiagnosticArray':
                return self.format_diagnostic_array(msg)
            elif msg_type == 'ChargingStatus':
                return self.format_charging_status(msg)
            else:
                # Generic fallback using reflection
                return self.format_generic_message(msg)
                
        except Exception as e:
            return f"  [Error formatting message: {str(e)}]"
        
    def on_topic_changed(self, topic_name):
        """Handle topic name changes and update publishers"""
        self.cleanup_publishers()
        self.setup_publishers(topic_name)

    def format_battery_state(self, msg):
        """Format BatteryState message"""
        power_supply_status = {
            0: 'Unknown', 1: 'Charging', 2: 'Discharging', 
            3: 'Not Charging', 4: 'Full'
        }
        power_supply_health = {
            0: 'Unknown', 1: 'Good', 2: 'Overheat', 3: 'Dead', 
            4: 'Overvoltage', 5: 'Unspecified Failure', 6: 'Cold', 
            7: 'Watchdog Timer Expire', 8: 'Safety Timer Expire'
        }
        power_supply_technology = {
            0: 'Unknown', 1: 'NiMH', 2: 'Li-ion', 3: 'Li-poly', 
            4: 'LiFe', 5: 'NiCd', 6: 'LiMn'
        }
        
        status_str = power_supply_status.get(msg.power_supply_status, f"Unknown({msg.power_supply_status})")
        health_str = power_supply_health.get(msg.power_supply_health, f"Unknown({msg.power_supply_health})")
        tech_str = power_supply_technology.get(msg.power_supply_technology, f"Unknown({msg.power_supply_technology})")
        
        return f"  voltage: {msg.voltage:.2f}V\n" \
            f"  current: {msg.current:.2f}A\n" \
            f"  charge: {msg.charge:.2f}Ah\n" \
            f"  capacity: {msg.capacity:.2f}Ah\n" \
            f"  design_capacity: {msg.design_capacity:.2f}Ah\n" \
            f"  percentage: {msg.percentage*100:.1f}%\n" \
            f"  power_supply_status: {status_str}\n" \
            f"  power_supply_health: {health_str}\n" \
            f"  power_supply_technology: {tech_str}\n" \
            f"  present: {msg.present}\n" \
            f"  location: '{msg.location}'\n" \
            f"  serial_number: '{msg.serial_number}'"

    def format_diagnostic_array(self, msg):
        """Format DiagnosticArray message"""
        level_names = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
        
        result = f"  header: frame_id='{msg.header.frame_id}', stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n"
        result += f"  status: [{len(msg.status)} items]\n"
        
        for i, status in enumerate(msg.status):
            level_str = level_names.get(status.level, f"Unknown({status.level})")
            result += f"    [{i}] name: '{status.name}'\n"
            result += f"        level: {level_str}\n"
            result += f"        message: '{status.message}'\n"
            result += f"        hardware_id: '{status.hardware_id}'\n"
            
            if status.values:
                result += f"        values: [{len(status.values)} items]\n"
                for j, kv in enumerate(status.values[:3]):
                    result += f"          [{j}] {kv.key}: {kv.value}\n"
                if len(status.values) > 3:
                    result += f"          ... and {len(status.values) - 3} more\n"
        
        return result

    def format_charging_status(self, msg):
        """Format ChargingStatus message"""
        fields = []
        
        # Use reflection to get all fields
        for field_name in dir(msg):
            if not field_name.startswith('_') and not callable(getattr(msg, field_name)):
                try:
                    value = getattr(msg, field_name)
                    if value is not None:
                        fields.append(f"  {field_name}: {value}")
                except:
                    pass
        
        return '\n'.join(fields) if fields else "  [No readable fields]"

    def format_generic_message(self, msg):
        """Generic message formatting using reflection"""
        fields = []
        
        # Try to get all public attributes
        for attr_name in dir(msg):
            if not attr_name.startswith('_') and not callable(getattr(msg, attr_name)):
                try:
                    value = getattr(msg, attr_name)
                    if hasattr(value, '__dict__'):  # Nested message
                        fields.append(f"  {attr_name}:")
                        for sub_attr in dir(value):
                            if not sub_attr.startswith('_') and not callable(getattr(value, sub_attr)):
                                try:
                                    sub_value = getattr(value, sub_attr)
                                    fields.append(f"    {sub_attr}: {sub_value}")
                                except:
                                    pass
                    else:
                        fields.append(f"  {attr_name}: {value}")
                except:
                    pass
        
        return '\n'.join(fields) if fields else "  [No readable fields]"


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
                
            self.message_type_status.setText(f"{self.current_message_type} publisher ready")
            self.message_type_status.setStyleSheet("color: green; font-size: 10px;")
            
        except Exception as e:
            print(f"Error creating publisher for {topic_name}: {str(e)}")
            self.message_type_status.setText(f"Publisher creation failed")
            self.message_type_status.setStyleSheet("color: red; font-size: 10px;")

    def cleanup_publishers(self):
        """Clean up existing publishers"""
        if self.twist_publisher:
            try:
                self.twist_publisher.destroy()  # Change from self.main_window.node.destroy_publisher()
            except Exception as e:
                print(f"Error destroying Twist publisher: {str(e)}")
            self.twist_publisher = None

        if self.twist_stamped_publisher:
            try:
                self.twist_stamped_publisher.destroy()  # Change from self.main_window.node.destroy_publisher()
            except Exception as e:
                print(f"Error destroying TwistStamped publisher: {str(e)}")
            self.twist_stamped_publisher = None

    def cleanup_data_subscribers(self):
        """Clean up all data monitor subscribers"""
        for topic_name, subscriber in self.data_subscribers.items():
            subscriber.destroy()
        self.data_subscribers.clear()
        self.topic_data.clear()
        self.topic_stats.clear()

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
            self.message_type_status.setText(f"Publishing error")
            self.message_type_status.setStyleSheet("color: red; font-size: 10px;")

    def update_available_topics(self, topics):
        """Update available topics in combo boxes - enhanced for better auto-detection"""
        self.available_topics = topics
        
        # Update monitor tab combo with all available topics
        if hasattr(self, 'monitor_topic_combo'):
            current_monitor_text = self.monitor_topic_combo.currentText()
            self.monitor_topic_combo.clear()
            
            # Sort topics for better organization
            sorted_topics = sorted(topics)
            
            # Add topics with their actual names as both display and data
            for topic in sorted_topics:
                self.monitor_topic_combo.addItem(topic, topic)  # Both display and data are the same
            
            # Restore previous selection
            if current_monitor_text:
                index = self.monitor_topic_combo.findText(current_monitor_text)
                if index >= 0:
                    self.monitor_topic_combo.setCurrentIndex(index)
                else:
                    self.monitor_topic_combo.setCurrentText(current_monitor_text)

    def is_compressed_topic(self, topic_name):
        """Check if a topic is a compressed image topic"""
        return (topic_name.endswith('/compressed') or 
                '/compressed' in topic_name or
                'compressed' in topic_name.lower())

    def get_selected_monitor_topic(self):
        """Get the actual topic name from the monitor combo box selection"""
        current_text = self.monitor_topic_combo.currentText().strip()
        
        # If there's a current selection, try to get the data
        current_index = self.monitor_topic_combo.currentIndex()
        if current_index >= 0:
            topic_data = self.monitor_topic_combo.itemData(current_index)
            if topic_data:
                return topic_data
        
        # If no data or manual entry, return the text directly
        # Remove any extra whitespace and validate
        if current_text and current_text.startswith('/'):
            return current_text
        elif current_text and not current_text.startswith('/'):
            return f"/{current_text}"  # Add leading slash if missing
        
        return current_text


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
            try:
                self.publish_twist()
            except Exception as e:
                print(f"Error sending stop command: {e}")
            
            # Clean up publishers
            self.cleanup_publishers()

    def closeEvent(self, event):
        """Clean up when the widget is closed"""
        if not self.destroyed:
            self.destroyed = True
            self.publish_timer.stop()
            
            # Send stop command before cleanup
            self.current_twist = (0.0, 0.0)
            if self.enable_control_btn.isChecked():
                self.publish_twist()
            
            self.cleanup_publishers()
            self.cleanup_data_subscribers()
            
            if hasattr(self, 'monitor_update_timer'):
                self.monitor_update_timer.stop()
                
        super().closeEvent(event)