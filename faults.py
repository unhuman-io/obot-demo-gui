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
    QMenuBar,
    QMenu,
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
    signal = pyqtSignal(str, str)
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
        self.signal.emit(self.combo_box.currentText(), self.text_widget.text())

class MotorTab(QWidget):
    def __init__(self, *args, **kwargs):
        super(MotorTab, self).__init__(*args, **kwargs)
        self.setAutoFillBackground(True)
        self.update_time = 100

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)

    def update(self):
        global motor_manager
        #print("update: " + self.name)
        self.status = motor_manager.read()[0]

    def pause(self):
        self.timer.stop()

    def unpause(self):
        self.timer.start(self.update_time)

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
            self.statuses[i].signal.connect(self.valueEdit)
            self.statuses[i].combo_box.addItems(self.field_names)
        self.setLayout(layout)


    def valueEdit(self, s, v):
        print("value edit: {}={}".format(s,v))
        if v:
            motor_manager.motors()[0][s] = v

    def update(self):
        super(StatusTab, self).update()
        for status in self.statuses:
            if not status.text_widget.hasFocus():
                val = motor_manager.motors()[0][status.combo_box.currentText()].get()
                status.setText(val)


class PlotTab(MotorTab):
    global motor_manager

    def __init__(self):
        super(PlotTab, self).__init__()

        self.name = "plot"
        self.update_time = 10

        self.chart = QChart()
        self.chart_view = QChartView(self.chart)
        self.chart_view.setRubberBand(QChartView.VerticalRubberBand)
        self.layout = QVBoxLayout()
        self.combo_box = QComboBox()
        self.combo_box.addItems(["motor_position", "joint_position", "iq"])
        self.layout.addWidget(self.combo_box)
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
        self.axis_y.setRange(-100,100)

        s = motor_manager.read()[0]
        self.mcu_timestamp = s.mcu_timestamp
        self.t_seconds = 0.0
        self.series.append(0, getattr(s, self.combo_box.currentText()))


    def update(self):
        super(PlotTab, self).update()
        s = self.status
        self.t_seconds += (motor.diff_mcu_time(s.mcu_timestamp, self.mcu_timestamp))/170e6
        self.mcu_timestamp = s.mcu_timestamp
        self.series.append(self.t_seconds, getattr(s, self.combo_box.currentText()))
        if len(self.series) > 500:
            self.series.remove(0)
        self.axis_x.setMax(self.t_seconds)
        self.axis_x.setMin(self.series.at(0).x())


class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        global motor_manager
        motor_manager = motor.MotorManager()
        motors = motor_manager.get_connected_motors(connect=False)
        print(motors)
        self.menu_bar = QMenuBar(self)
        self.motor_menu = QMenu("&Motor")
        self.menu_bar.addMenu(self.motor_menu)
        self.setMenuBar(self.menu_bar)
        for m in motors:
            action = self.motor_menu.addAction(m.name())
            action.triggered.connect(lambda: self.connect_motor(m.name()))
        self.connect_motor(motors[0].name())


        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.West)

        self.tabs.addTab(FaultTab(), "fault")
        self.tabs.addTab(StatusTab(), "status")
        self.tabs.addTab(PlotTab(), "plot")

        self.setCentralWidget(self.tabs)
        self.last_tab = self.tabs.currentWidget()
        self.last_tab.unpause()

        self.tabs.currentChanged.connect(self.new_tab)
        if "-fullscreen" in QCoreApplication.arguments():
            self.showFullScreen()

    def connect_motor(self, name):
         print("Connecting motor " + name)
         motor_manager.get_motors_by_name([name])
         self.setWindowTitle(name)

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
