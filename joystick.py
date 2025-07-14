import sys
import math
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QMainWindow)
from PyQt6.QtCore import Qt, QPoint, QRect, QTimer, pyqtSignal
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QPixmap, QIcon, QFont

class CustomJoystick(QWidget):
    # Signals for joystick events
    positionChanged = pyqtSignal(float, float)  # x, y values from -1 to 1
    buttonPressed = pyqtSignal(str)  # button name
    buttonReleased = pyqtSignal(str)  # button name
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self.setMaximumSize(500, 500)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        # Button properties
        self.buttons = {}
        self.button_states = {}
        self.custom_icons = {}
        self.v_x = 0 
        self.v_y = 0
        
        # Keyboard state
        self.pressed_keys = set()
        
        # Initialize default buttons
        self._init_default_buttons()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)

        
        # Timer for smooth movement
        self.move_timer = QTimer()
        self.move_timer.start(16)  # ~60 FPS
        
        # Movement speed
        self.move_speed = 3
        
    def _init_default_buttons(self):
        """Initialize default button layout"""
        # Define button positions and default labels
        button_config = {
            'Up': {'key': Qt.Key.Key_Up, 'color': QColor(255, 84, 0)},
            'Down': {'key': Qt.Key.Key_Down, 'color': QColor(0, 180, 216)},
            'Left': {'key': Qt.Key.Key_Left, 'color': QColor(0, 0, 255)},
            'Right': {'key': Qt.Key.Key_Right, 'color': QColor(255, 0, 0)},
        }
        
        for name, config in button_config.items():
            self.buttons[name] = {
                'pos': QPoint(0, 0),  # Will be set in resizeEvent
                'key': config['key'],
                'radius': 30,
                'color': config['color']
            }
            self.button_states[name] = False

        self.update_button_positions()
    
    def keyPressEvent(self, event):
        """Handle key press events"""
        if event.key() in self.pressed_keys:
            return
            
        self.pressed_keys.add(event.key())
        
        # Check button presses
        for button_name, button_data in self.buttons.items():
            if event.key() == button_data['key']:
                self.button_states[button_name] = True
                self.buttonPressed.emit(button_name)
                self.update()
                break
        
        # Handle directional movement
        self._handle_movement()
        self.update()
    
    def keyReleaseEvent(self, event):
        """Handle key release events"""
        if event.key() in self.pressed_keys:
            self.pressed_keys.remove(event.key())
        
        # Check button releases
        for button_name, button_data in self.buttons.items():
            if event.key() == button_data['key']:
                self.button_states[button_name] = False
                self.buttonReleased.emit(button_name)
                self.update()
                break
        
        # Handle directional movement
        self._handle_movement()
        self.update()
    
    def _handle_movement(self):
        """Handle joystick movement based on pressed keys"""
        # Default movement keys
        self.v_x = 0
        self.v_y = 0
        
        if Qt.Key.Key_Left in self.pressed_keys or Qt.Key.Key_A in self.pressed_keys:
            self.v_x -= self.move_speed
        if Qt.Key.Key_Right in self.pressed_keys or Qt.Key.Key_D in self.pressed_keys:
            self.v_x += self.move_speed
        if Qt.Key.Key_Up in self.pressed_keys or Qt.Key.Key_W in self.pressed_keys:
            self.v_y += self.move_speed
        if Qt.Key.Key_Down in self.pressed_keys or Qt.Key.Key_S in self.pressed_keys:
            self.v_y -= self.move_speed
        self.positionChanged.emit(self.v_x, self.v_y)
        
    
    
    def paintEvent(self, event):
        """Paint the joystick and buttons"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        for button_name, button_data in self.buttons.items():
            pos = button_data['pos']
            radius = button_data['radius']
            color = button_data['color']
            
            if self.button_states[button_name]:
                painter.setPen(QPen(QColor(255, 255, 255), 3))
                painter.setBrush(QBrush(color.lighter(150)))
            else:
                painter.setPen(QPen(QColor(0, 0, 0), 2))
                painter.setBrush(QBrush(color))
            #elipse
            painter.drawEllipse(pos.x() - radius, pos.y() - radius, 
                              radius * 2, radius * 2)
            
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            font_size = max(8, radius // 4)  
            painter.setFont(QFont("Arial", font_size, QFont.Weight.Bold))
            text_rect = QRect(pos.x() - radius, pos.y() - radius//3, radius * 2, radius)
            painter.drawText(text_rect, Qt.AlignmentFlag.AlignCenter, button_name)

    def update_button_positions(self):
        """Update button positions based on current widget size"""
        center_x = self.width() // 2
        center_y = self.height() // 2
        
        # Calculate button distance based on widget size
        button_distance = min(self.width(), self.height()) // 4
        button_radius = min(self.width(), self.height()) // 12
        
        # Update positions and radius for all buttons
        for button_name in self.buttons:
            self.buttons[button_name]['radius'] = button_radius
            
        self.buttons['Up']['pos'] = QPoint(center_x, center_y - button_distance)
        self.buttons['Down']['pos'] = QPoint(center_x, center_y + button_distance)
        self.buttons['Left']['pos'] = QPoint(center_x - button_distance, center_y)
        self.buttons['Right']['pos'] = QPoint(center_x + button_distance, center_y)

    def resizeEvent(self, event):
        """Handle widget resize to update button positions"""
        super().resizeEvent(event)
        if hasattr(self, 'buttons') and self.buttons:  # Only if buttons are initialized
            self.update_button_positions()
            self.update()


