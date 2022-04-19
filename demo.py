#!/usr/bin/env python3

import sys
sys.path.append("/usr/share/motor-realtime")

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
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
)
from PyQt6.QtGui import QPalette, QColor, QDoubleValidator
import motor

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

motor_manager = None

class NumberEdit(QWidget):
    def __init__(self, name):
        super(NumberEdit, self).__init__()
        layout = QHBoxLayout()
        self.name = name
        layout.addWidget(QLabel(name))
        self.number_widget = QLineEdit(name)
        layout.addWidget(self.number_widget)
        self.setLayout(layout)
    
    def setNumber(self, number):
        self.number_widget.setText(str(number))

class NumberDisplay(NumberEdit):
    def __init__(self, *args, **kwargs):
        super(NumberDisplay, self).__init__(*args, **kwargs)
        self.number_widget.setReadOnly(True)

class DataDisplay(QWidget):
    def __init__(self):
        super(DataDisplay, self).__init__()
        self.setAutoFillBackground(True)

        self.name = "data"
        self.fields = ["mcu_timestamp", "motor_position", "joint_position", "iq", 
            "torque"]
        self.numbers = []
        layout = QVBoxLayout()       
        
        self.freeze = QPushButton(self)
        self.freeze.clicked.connect(self.pause)
        self.freeze.setText("Pause")
        layout.addWidget(self.freeze)

        for field in self.fields:
            widget = NumberDisplay(field)
            self.numbers.append(widget)
            layout.addWidget(widget)
        

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def pause(self):
        if self.timer.isActive():
            self.timer.stop()
        else:
            self.timer.start(100)

    def update(self):
        global motor_manager
        s = motor_manager.read()[0]
        for widget in self.numbers:
            widget.setNumber(getattr(s,widget.name))

class Position(QWidget):
    def __init__(self):
        super(Position, self).__init__()
        self.setAutoFillBackground(True)

        self.name = "position"
        self.numbers = []
        layout = QVBoxLayout()
        self.widget = NumberEdit("position")
        self.widget.number_widget.editingFinished.connect(self.position_update)
        self.widget.number_widget.setValidator(QDoubleValidator())       
        layout.addWidget(self.widget)
        self.setLayout(layout)

    def position_update(self):
        p = float(self.widget.number_widget.text())
        print("position command " + str(p))
        motor_manager.set_command_position([p])
        motor_manager.set_command_mode(motor.ModeDesired.Position)
        motor_manager.write_saved_commands()

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

        self.tabs.addTab(DataDisplay(), "data")
        self.tabs.addTab(Position(), "position")

        self.setCentralWidget(self.tabs)
        self.last_tab = self.tabs.currentWidget()

        self.tabs.currentChanged.connect(self.new_tab)

    def new_tab(self, index):
        print("last tab " + str(index) + " " + self.last_tab.name)
        print("new tab " + str(index) + " " + self.tabs.widget(index).name)
        self.last_tab = self.tabs.widget(index)
    




app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()
