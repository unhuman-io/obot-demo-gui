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
    QComboBox,
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

class StatusCombo(QWidget):
    signal = pyqtSignal(float)
    def __init__(self, *args, **kwargs):
        super(StatusCombo, self).__init__(*args, **kwargs)
        
        self.layout = QHBoxLayout()
        self.combo_box = QComboBox()
        self.layout.addWidget(self.combo_box)
        self.text_widget = QLineEdit()
        self.text_widget.editingFinished.connect(self.editingFinished)
        self.layout.addWidget(self.text_widget)
        self.setLayout(self.layout)

    
    def setText(self, number):
        self.text_widget.setText(str(number))

    def editingFinished(self):
        self.signal.emit(float(self.text_widget.text()))

class MotorTab(QWidget):
    def __init__(self, *args, **kwargs):
        super(MotorTab, self).__init__(*args, **kwargs)
        self.setAutoFillBackground(True)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)

    def update(self):
        global motor_manager
        #print("update: " + self.name)
        self.status = motor_manager.read()[0]

    def pause(self):
        self.timer.stop()

    def unpause(self):
        self.timer.start(100)

class FaultTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(FaultTab, self).__init__(*args, **kwargs)
        
        self.name = "fault"
        #print("init: " + self.name)
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

    def update(self):
        super(FaultTab, self).update()
        
        self.faults[0].setText("mode: " + self.status.flags.mode.name.lower())
        faults = self.status.flags.error.bits
        for widget in self.faults:
            if widget.text() in faults.keys():
                if faults[widget.text()]:
                    widget.setStyleSheet("background-color: red")
                else:
                    widget.setStyleSheet("background-color: green")

class StatusTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(StatusTab, self).__init__(*args, **kwargs)
        self.setAutoFillBackground(True)

        self.name = "status"
        #print("init: " + self.name)
        self.field_names = motor_manager.motors()[0]["help"].get().split('\n')

        self.statuses = []
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)    
        layout.setContentsMargins(QMargins(0,0,0,0))
        for i in range(5):
            self.statuses.append(StatusCombo())
            layout.addWidget(self.statuses[i])
            self.statuses[i].combo_box.addItems(self.field_names)
        self.setLayout(layout)

    def update(self):
        super(StatusTab, self).update()
        for status in self.statuses:
            val = motor_manager.motors()[0][status.combo_box.currentText()].get()
            status.setText(val)

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

        self.tabs.addTab(FaultTab(), "fault")
        self.tabs.addTab(StatusTab(), "status")

        self.setCentralWidget(self.tabs)
        self.last_tab = self.tabs.currentWidget()
        self.last_tab.unpause()

        self.tabs.currentChanged.connect(self.new_tab)
        if "-fullscreen" in QCoreApplication.arguments():
            self.showFullScreen()

    def new_tab(self, index):
        #print("last tab " + str(index) + " " + self.last_tab.name)
        self.last_tab.pause()
        #print("new tab " + str(index) + " " + self.tabs.widget(index).name)
        self.tabs.widget(index).unpause()
        self.last_tab = self.tabs.widget(index)


app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()
