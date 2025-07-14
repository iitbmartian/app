from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
                            QComboBox, QScrollArea, QGridLayout, QGroupBox, QSpinBox)
from PyQt6.QtCore import Qt
from image_stuff import CameraWidget
from ros_utils import ImageSubscriber, CompressedImageSubscriber



class ImageTab(QWidget):
    def __init__(self, parent_signals, main_window, num_cams) :
        super().__init__()
        self.signals = parent_signals # Signals like mouse click, int etc
        self.main_window = main_window
        self.signals.image_received.connect(self.update_camera_image)
        
        self.max_cameras = num_cams
        self.camera_widgets = []
        self.camera_topics = [None] * self.max_cameras
        self.available_topics = []
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Control panel
        control_group = QGroupBox("Camera Controls")
        control_layout = QVBoxLayout(control_group)
        
        # Number of cameras
        camera_count_layout = QHBoxLayout()
        camera_count_layout.addWidget(QLabel("Active Cameras:"))
        self.camera_count_spin = QSpinBox() # The counter to pick number of cameras
        self.camera_count_spin.setMinimum(1)
        self.camera_count_spin.setMaximum(self.max_cameras)
        self.camera_count_spin.setValue(2)
        self.camera_count_spin.valueChanged.connect(self.update_camera_layout) # get updated value when u change the spinbox
        camera_count_layout.addWidget(self.camera_count_spin)
        camera_count_layout.addStretch()
        
        control_layout.addLayout(camera_count_layout)
        
        # Camera topic selection
        self.topic_layouts = []
        for i in range(self.max_cameras+1):
            topic_layout = QHBoxLayout()
            
            label = QLabel(f"Camera {i+1}:")
            label.setMinimumWidth(80)
            topic_layout.addWidget(label)
            
            combo = QComboBox() # Combobox is the drop-down menu
            combo.setEditable(True)
            combo.setPlaceholderText("No image topics available")
            topic_layout.addWidget(combo)
            
            connect_btn = QPushButton("Connect")
            connect_btn.clicked.connect(lambda checked, idx=i: self.main_window.connect_image_camera(idx))
            topic_layout.addWidget(connect_btn)
            
            disconnect_btn = QPushButton("Disconnect")
            disconnect_btn.clicked.connect(lambda checked, idx=i: self.main_window.disconnect_image_camera(idx))
            topic_layout.addWidget(disconnect_btn)
            
            # Disable buttons initially
            connect_btn.setEnabled(False)
            disconnect_btn.setEnabled(False)
            
            combo.camera_d = i
            connect_btn.camera_id = i
            disconnect_btn.camera_id = i
            
            control_layout.addLayout(topic_layout)
            self.topic_layouts.append((label, combo, connect_btn, disconnect_btn))
        
        layout.addWidget(control_group)
        
        # Camera display area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        
        self.camera_display_widget = QWidget()
        self.camera_grid_layout = QGridLayout(self.camera_display_widget)
        self.camera_grid_layout.setSpacing(10) # space between video streams
        
        # Create camera widgets
        for i in range(self.max_cameras):
            camera_widget = CameraWidget(i)
            self.camera_widgets.append(camera_widget)
            camera_widget.hide()
        
        scroll_area.setWidget(self.camera_display_widget)
        layout.addWidget(scroll_area, 1)
        
        self.update_camera_layout()

    def update_available_topics(self, topics):
        """Update the available topics in combo boxes"""
        self.available_topics = topics
        
        for _, combo, connect_btn, disconnect_btn in self.topic_layouts:
            current_text = combo.currentText()
            combo.clear()
            
            if topics:
                combo.addItems(topics)
                combo.setPlaceholderText("Select image topic...")
                connect_btn.setEnabled(True)
            else:
                combo.setPlaceholderText("No image topics available")
                connect_btn.setEnabled(False)
            
            # Restore previous selection if it still exists
            if current_text and current_text in topics:
                combo.setCurrentText(current_text)

    def update_camera_layout(self):
        num_cameras = self.camera_count_spin.value()
        
        for widget in self.camera_widgets:
            widget.hide()
        
        for i in reversed(range(self.camera_grid_layout.count())):
            self.camera_grid_layout.itemAt(i).widget().setParent(None)
        
        if num_cameras == 1:
            rows, cols = 1, 1
        elif num_cameras == 2:
            rows, cols = 1, 2
        elif num_cameras <= 4:
            rows, cols = 2, 2
        else:
            rows, cols = 2, 3
        
        for i in range(num_cameras):
            row = i // cols
            col = i % cols
            self.camera_grid_layout.addWidget(self.camera_widgets[i], row, col)
            self.camera_widgets[i].show()
        
        for i, (label, combo, connect_btn, disconnect_btn) in enumerate(self.topic_layouts):
            visible = i < num_cameras
            label.setVisible(visible)
            combo.setVisible(visible)
            connect_btn.setVisible(visible)
            disconnect_btn.setVisible(visible)

    def update_camera_image(self, cv_image, camera_id):
        if camera_id < len(self.camera_widgets):
            self.camera_widgets[camera_id].update_image(cv_image)

    def connect_camera(self, camera_id, image_subscribers, callback):
        """Connect to selected image topic for specific camera"""
        _, combo, connect_btn, disconnect_btn = self.topic_layouts[camera_id]
        topic_name = combo.currentText().strip()

        if not topic_name or topic_name not in self.available_topics:
            return False

        try:
            # Disconnect existing subscriber for this camera
            if self.camera_topics[camera_id] in image_subscribers:
                old_subscriber = image_subscribers[self.camera_topics[camera_id]]
                if old_subscriber:
                    old_subscriber.destroy_node()
                del image_subscribers[self.camera_topics[camera_id]]

            # Create new subscriber
            # Determine if this is a compressed image topic
            is_compressed = self.is_compressed_topic(topic_name)

            # Create new subscriber based on message type
            if is_compressed:
                subscriber = CompressedImageSubscriber(topic_name, callback, camera_id)
            else:
                subscriber = ImageSubscriber(topic_name, callback, camera_id)            
            image_subscribers[topic_name] = subscriber
            self.camera_topics[camera_id] = topic_name

            # Update UI
            self.camera_widgets[camera_id].set_topic(topic_name)
            connect_btn.setEnabled(False)
            disconnect_btn.setEnabled(True)
            
            return True

        except Exception as e:
            print(f"Error connecting camera {camera_id} to topic {topic_name}: {str(e)}")
            return False

    def disconnect_camera(self, camera_id, image_subscribers):
        """Disconnect from current topic for specific camera"""
        _, combo, connect_btn, disconnect_btn = self.topic_layouts[camera_id]

        topic_name = self.camera_topics[camera_id]
        if topic_name and topic_name in image_subscribers:
            subscriber = image_subscribers[topic_name]
            if subscriber:
                subscriber.destroy_node()
            del image_subscribers[topic_name]

        self.camera_topics[camera_id] = None
        self.camera_widgets[camera_id].set_topic(None)

        connect_btn.setEnabled(True)
        disconnect_btn.setEnabled(False)
        
        return True
    
    def is_compressed_topic(self, topic_name):
        """Check if a topic is a compressed image topic"""
        return (topic_name.endswith('/compressed') or 
                '/compressed' in topic_name or
                'compressed' in topic_name.lower())
