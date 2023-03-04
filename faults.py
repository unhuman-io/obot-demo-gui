#!/usr/bin/env python3

import sys


from PyQt5.QtCore import QTimer, Qt, QMargins, QCoreApplication, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QPushButton,
    QTabWidget,
    QWidget,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QLineEdit,
    QSlider,
)
from PyQt5.QtGui import QPalette, QColor, QDoubleValidator
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
import motor

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

motor_manager = None

class StatusDisplay(QPushButton):
    def __init__(self, *args, **kwargs):
        super(StatusDisplay, self).__init__(*args, **kwargs)

    def mousePressEvent(self, event):
        pass

    def keyPressEvent(self, event):
        pass

class FaultDisplay(QWidget):
    def __init__(self):
        super(FaultDisplay, self).__init__()
        self.setAutoFillBackground(True)

        self.name = "fault"
        mask = motor_manager.motors()[0].error_mask()
        self.fields = ["mode", *list(mask.keys())]
        self.faults = []
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)    
        layout.setContentsMargins(QMargins(0,0,0,0))
        

        for field in self.fields:
            widget = StatusDisplay(field)
            if field in mask.keys():
                if not mask[field]:
                    widget.setDisabled(True)
            
            self.faults.append(widget)
            layout.addWidget(widget)
        
        layout.setSpacing(0)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def update(self):
        global motor_manager
        s = motor_manager.read()[0]
        self.faults[0].setText("mode: " + s.flags.mode.name.lower())
        faults = s.flags.error.bits
        for widget in self.faults:
            if widget.text() in faults.keys():
                if faults[widget.text()]:
                    widget.setStyleSheet("background-color: red")
                else:
                    widget.setStyleSheet("background-color: green")

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        global motor_manager
        motor_manager = motor.MotorManager()
        motors = motor_manager.get_connected_motors()
        m1 = motors[0]

        self.setWindowTitle(m1.name())
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.West)

        self.tabs.addTab(FaultDisplay(), "fault")

        self.setCentralWidget(self.tabs)
        self.last_tab = self.tabs.currentWidget()

        self.tabs.currentChanged.connect(self.new_tab)
        if "-fullscreen" in QCoreApplication.arguments():
            self.showFullScreen()

    def new_tab(self, index):
        print("last tab " + str(index) + " " + self.last_tab.name)
        print("new tab " + str(index) + " " + self.tabs.widget(index).name)
        self.last_tab = self.tabs.widget(index)


app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()
