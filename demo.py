#!/usr/bin/env python3

import sys
sys.path.append("/usr/share/motor-realtime")

from PyQt5.QtCore import QTimer, Qt, QMargins, QCoreApplication
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
)
from PyQt5.QtGui import QPalette, QColor, QDoubleValidator
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
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
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)    
        layout.setContentsMargins(QMargins(0,0,0,0))
        
        self.freeze = QPushButton(self)
        self.freeze.clicked.connect(self.pause)
        self.freeze.setText("Pause")
        layout.addWidget(self.freeze)

        for field in self.fields:
            widget = NumberDisplay(field)
            self.numbers.append(widget)
            layout.addWidget(widget)
        
        layout.setSpacing(0)
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
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
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

class Plot(QWidget):
    global motor_manager

    def __init__(self):
        super(Plot, self).__init__()
        self.setAutoFillBackground(True)
        self.name = "plot"

        self.chart = QChart()
        self.chart_view = QChartView(self.chart)
        self.layout = QHBoxLayout()
        self.layout.addWidget(self.chart_view)
        self.setLayout(self.layout)

        self.series = QLineSeries()
        self.chart.addSeries(self.series)
        self.axis_x = QValueAxis()
        self.axis_x.setTickCount(10)
        self.axis_x.setTitleText("Time (s)")
        self.chart.addAxis(self.axis_x, Qt.AlignmentFlag.AlignBottom)
        self.series.attachAxis(self.axis_x)
        self.axis_y = QValueAxis()
        self.axis_y.setTickCount(10)
        self.axis_y.setTitleText("Value")
        self.chart.addAxis(self.axis_y, Qt.AlignmentFlag.AlignLeft)
        self.series.attachAxis(self.axis_y)
        self.axis_y.setRange(-10,10)

        s = motor_manager.read()[0]
        self.mcu_timestamp = s.mcu_timestamp
        self.t_seconds = 0.0
        self.series.append(0, s.iq)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)


    def update(self):
        s = motor_manager.read()[0]
        self.t_seconds += (motor.diff_mcu_time(s.mcu_timestamp, self.mcu_timestamp))/170e6
        self.mcu_timestamp = s.mcu_timestamp
        self.series.append(self.t_seconds, s.iq)
        if len(self.series) > 500:
            self.series.remove(0)
        self.axis_x.setMax(self.t_seconds)
        self.axis_x.setMin(self.series.at(0).x())

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
        self.tabs.addTab(Plot(), "plot")

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
