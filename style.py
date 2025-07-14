from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QFont

def style():
    app = QApplication.instance()
    font = QFont("Segoe UI", 10)
    app.setFont(font)
    
    modern_style = """
    /* Main Window */
    QMainWindow {
        background-color: #1e1e1e;
        color: #ffffff;
    }
    
    /* Central Widget */
    QWidget {
        background-color: #1e1e1e;
        color: #ffffff;
    }
    
    /* Tab Widget */
    QTabWidget {
        background-color: #1e1e1e;
        border: none;
    }
    
    QTabWidget::pane {
        border: 2px solid #3c3c3c;
        background-color: #2d2d2d;
        border-radius: 8px;
        margin-top: 5px;
    }
    
    QTabBar::tab {
        background-color: #404040;
        color: #ffffff;
        padding: 12px 20px;
        margin-right: 4px;
        margin-bottom: 2px;
        border-top-left-radius: 8px;
        border-top-right-radius: 8px;
        min-width: 120px;
        font-weight: 500;
    }
    
    QTabBar::tab:selected {
        background-color: #0078d4;
        color: #ffffff;
        border: 2px solid #106ebe;
        border-bottom: none;
    }
    
    QTabBar::tab:hover:!selected {
        background-color: #505050;
    }
    
    /* Buttons */
    QPushButton {
        background-color: #0078d4;
        color: #ffffff;
        border: none;
        padding: 10px 20px;
        border-radius: 6px;
        font-weight: 500;
        min-height: 16px;
    }
    
    QPushButton:hover {
        background-color: #106ebe;
    }
    
    QPushButton:pressed {
        background-color: #005a9e;
    }
    
    QPushButton:disabled {
        background-color: #404040;
        color: #808080;
    }
    
    /* Refresh Button Special Styling */
    QPushButton#refreshButton {
        background-color: #107c10;
        padding: 12px 24px;
        font-weight: 600;
    }
    
    QPushButton#refreshButton:hover {
        background-color: #0e6b0e;
    }
    
    /* Group Box */
    QGroupBox {
        background-color: #2d2d2d;
        border: 2px solid #3c3c3c;
        border-radius: 8px;
        margin-top: 8px;
        padding-top: 10px;
        font-weight: 600;
        font-size: 11px;
    }
    
    QGroupBox::title {
        subcontrol-origin: margin;
        subcontrol-position: top left;
        padding: 0 8px;
        background-color: #2d2d2d;
        color: #ffffff;
    }
    
    /* Text Edit */
    QTextEdit {
        background-color: #262626;
        border: 2px solid #3c3c3c;
        border-radius: 6px;
        padding: 10px;
        color: #ffffff;
        font-family: "Consolas", "Monaco", monospace;
        font-size: 10px;
        line-height: 1.4;
    }
    
    QTextEdit:focus {
        border: 2px solid #0078d4;
    }
    
    /* Splitter */
    QSplitter::handle {
        background-color: #3c3c3c;
        width: 3px;
        margin: 2px;
        border-radius: 1px;
    }
    
    QSplitter::handle:hover {
        background-color: #0078d4;
    }
    
    /* Scrollbar */
    QScrollBar:vertical {
        background-color: #2d2d2d;
        width: 12px;
        border-radius: 6px;
        margin: 0px;
    }
    
    QScrollBar::handle:vertical {
        background-color: #505050;
        min-height: 20px;
        border-radius: 6px;
        margin: 2px;
    }
    
    QScrollBar::handle:vertical:hover {
        background-color: #606060;
    }
    
    QScrollBar::add-line:vertical,
    QScrollBar::sub-line:vertical {
        height: 0px;
    }
    
    QScrollBar:horizontal {
        background-color: #2d2d2d;
        height: 12px;
        border-radius: 6px;
        margin: 0px;
    }
    
    QScrollBar::handle:horizontal {
        background-color: #505050;
        min-width: 20px;
        border-radius: 6px;
        margin: 2px;
    }
    
    QScrollBar::handle:horizontal:hover {
        background-color: #606060;
    }
    
    QScrollBar::add-line:horizontal,
    QScrollBar::sub-line:horizontal {
        width: 0px;
    }
    
    /* Status Bar */
    QStatusBar {
        background-color: #333333;
        color: #ffffff;
        border-top: 1px solid #3c3c3c;
        padding: 4px;
        font-size: 10px;
    }
    
    /* Tooltip */
    QToolTip {
        background-color: #404040;
        color: #ffffff;
        border: 1px solid #606060;
        border-radius: 4px;
        padding: 4px;
        font-size: 10px;
    }
    
    /* Line Edit (if any) */
    QLineEdit {
        background-color: #262626;
        border: 2px solid #3c3c3c;
        border-radius: 6px;
        padding: 8px;
        color: #ffffff;
        font-size: 10px;
    }
    
    QLineEdit:focus {
        border: 2px solid #0078d4;
    }
    
    /* ComboBox (if any) */
    QComboBox {
        background-color: #262626;
        border: 2px solid #3c3c3c;
        border-radius: 6px;
        padding: 8px;
        color: #ffffff;
        min-width: 100px;
    }
    
    QComboBox:focus {
        border: 2px solid #0078d4;
    }
    
    QComboBox::drop-down {
        border: none;
        width: 20px;
    }
    
    QComboBox::down-arrow {
        image: none;
        border-left: 5px solid transparent;
        border-right: 5px solid transparent;
        border-top: 5px solid #ffffff;
        margin-right: 5px;
    }
    
    QComboBox QAbstractItemView {
        background-color: #262626;
        border: 2px solid #3c3c3c;
        border-radius: 6px;
        color: #ffffff;
        selection-background-color: #0078d4;
    }
    """
    
    app.setStyleSheet(modern_style)