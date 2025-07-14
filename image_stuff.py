from PyQt6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout,
                            QHBoxLayout, QWidget, QLabel, QPushButton,
                            QComboBox, QTextEdit, QSplitter, QScrollArea,
                            QGridLayout, QFrame, QSpinBox, QGroupBox)
from PyQt6.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt6.QtGui import QImage, QPixmap, QFont
import cv2

class CameraWidget(QFrame):
    def __init__(self, camera_id):
        super().__init__()
        self.camera_id = camera_id
        self.setup_ui()

    def setup_ui(self):
        
        # Setting up the box's line width

        self.setFrameStyle(QFrame.Shape.Box)
        self.setLineWidth(2)

        layout = QVBoxLayout(self) # takes QWidget(self) as parent
        layout.setContentsMargins(5, 5, 5, 5) # Sets the left, top, right, and bottom margins to use around the layout.

        # Header with topic info
        self.header_label = QLabel(f"Camera {self.camera_id + 1}: Not Connected")
        self.header_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        font = QFont()
        font.setBold(True)
        self.header_label.setFont(font)
        self.header_label.setStyleSheet("background-color: #f0f0f0; padding: 5px; border: 1px solid #ccc;")
        layout.addWidget(self.header_label)

        # Image display
        self.image_label = QLabel()
        self.image_label.setStyleSheet("border: 1px solid gray; background-color: #2a2a2a;")
        self.image_label.setMinimumSize(200, 150)
        self.image_label.setScaledContents(True)
        self.image_label.setText("No Image")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("color: white; font-size: 12px; border: 1px solid gray; background-color: #2a2a2a;")
        layout.addWidget(self.image_label)

        # Info display
        self.info_label = QLabel("Disconnected")
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.info_label.setStyleSheet("font-size: 10px; color: #666;")
        layout.addWidget(self.info_label)

    def update_image(self, cv_image):
        try:
            # Convert BGR to RGB for Qt
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w

            # Create QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

            # Convert to QPixmap and display
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)

            # Update info
            self.info_label.setText(f"{w}x{h}")

        except Exception as e:
            self.info_label.setText(f"Error: {str(e)[:20]}")

    def set_topic(self, topic_name):
        if topic_name:
            self.header_label.setText(f"Camera {self.camera_id + 1}: {topic_name}")
            self.header_label.setStyleSheet("background-color: #d4edda; padding: 5px; border: 1px solid #c3e6cb; color: #155724;")
        else:
            self.header_label.setText(f"Camera {self.camera_id + 1}: Not Connected")
            self.header_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border: 1px solid #f5c6cb; color: #721c24;")
            self.image_label.setText("Disconnected")
            self.image_label.setPixmap(QPixmap())
            self.info_label.setText("Disconnected")
