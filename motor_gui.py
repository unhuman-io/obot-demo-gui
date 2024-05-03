#!/usr/bin/env python3

import sys
import traceback
import os
import glob
import re
import json
import yaml
import subprocess

# for other included files, so pyinstaller will build with it
import boto3
import git
import gspread

# for other included files, so pyinstaller will build with it
import boto3
import git
import gspread
import oauth2client
import oauth2client.service_account
import paramiko

from PyQt5.QtCore import QTimer, Qt, QMargins, QCoreApplication, pyqtSignal, QPointF, QEvent
from PyQt5.QtWidgets import (
    QAction,
    QApplication,
    QLabel,
    QMainWindow,
    QInputDialog,
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
    QCheckBox,
    QMenu,
    QPlainTextEdit,
    QFrame,
    QRadioButton,
    QFileDialog,
    QTextEdit,
    QMessageBox
)

from PyQt5.QtGui import QPalette, QColor, QDoubleValidator
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QScatterSeries, QValueAxis, QLogValueAxis

import motor
import numpy as np
from io import StringIO
import time
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

session = boto3.Session()
motor_manager = None

# Read the environment variables to get the path to project-x
project_path = os.getenv('PROJECT_PATH')
if project_path is not None:
    sys.path.append(project_path + "/tools/calibration")
    from motor_handler import MotorHandler
    from aws_server import S3Server
    import utils

def current_motor():
    return motor_manager.motors()[0]

cpu_frequency = 170e6

def mode_open():
    print("open")
    motor_manager.set_command_mode(motor.ModeDesired.Open)
    motor_manager.write_saved_commands()

def is_ip_address(s):
    # Define the regex pattern for an IP address or IP-like string with 'X' as a placeholder
    pattern = r'^(\d{1,3}\.){3}(\d{1,3}|X)(:\d{1,6})?$'
    
    # Use re.match to check if the string matches the pattern
    if re.match(pattern, s):
        return True  # The string is an IP address or an IP-like string
    else:
        return False  # The string does not match the pattern

class NumberEdit(QWidget):
    signal = pyqtSignal(float)
    def __init__(self, name, description=None, tooltip=None, value=0):
        super(NumberEdit, self).__init__()

        if description is None:
            description = name
        self.layout = QHBoxLayout()
        self.name = name
        self.label = QLabel(description)
        if tooltip is None:
            tooltip = name
        self.label.setToolTip(tooltip)
        self.layout.addWidget(self.label)
        self.number_widget = QLineEdit()
        self.number_widget.setValidator(QDoubleValidator())
        self.setNumber(value)
        self.number_widget.editingFinished.connect(self.editingFinished)
        self.layout.addWidget(self.number_widget)
        self.setLayout(self.layout)

    def setNumber(self, number):
        num = str(number)
        try:
            num = str(int(num))
        except ValueError:
            try:
                num = "{:.4f}".format(float(num))
            except ValueError:
                pass
        self.number_widget.setText(num)

    def getNumber(self):
        val = 0.0
        try:
            val = float(self.number_widget.text())
        except ValueError:
            pass
        return val

    def editingFinished(self):
        self.signal.emit(self.getNumber())

class NumberDisplay(NumberEdit):
    def __init__(self, *args, **kwargs):
        super(NumberDisplay, self).__init__(*args, **kwargs)
        self.number_widget.setReadOnly(True)



class APIEdit(NumberEdit):
    def __init__(self, name, description=None, tooltip=None, *args, **kwargs):
        if tooltip is None:
            tooltip = "api: " + name
        else:
            tooltip = "api: {}\n{}".format(name, tooltip)
        super(APIEdit, self).__init__(name, description, tooltip, *args, **kwargs)
        self.populate_value()
        
    def editingFinished(self):
        current_motor()[self.name] = self.number_widget.text()
        return super().editingFinished()
    
    def update(self):
        if not self.number_widget.hasFocus():
            self.populate_value()

    def populate_value(self):
        val = current_motor()[self.name].get()
        self.number_widget.setText(val)

class APIDisplay(APIEdit):
    def __init__(self, *args, **kwargs):
        super(APIDisplay, self).__init__(*args, **kwargs)
        self.number_widget.setReadOnly(True)

class APIBool(QWidget):
    def __init__(self, name, description=None, tooltip=None, *args, **kwargs):
        super(APIBool, self).__init__(*args, **kwargs)

        if description is None:
            description = name
        self.layout = QHBoxLayout()
        self.name = name
        self.checkbox = QCheckBox()
        self.layout.addWidget(self.checkbox)
        self.label = QLabel(description)
        if tooltip is None:
            tooltip = "api: " + name
        else:
            tooltip = "api: {}\n{}".format(name, tooltip)
        self.label.setToolTip(tooltip)
        self.layout.addWidget(self.label)
        self.checkbox.clicked.connect(self.clicked)
        self.setLayout(self.layout)

    def update(self):
        val = current_motor()[self.name].get()
        try:
            self.checkbox.setChecked(float(val) != 0)
        except ValueError:
            pass
    
    def clicked(self, on):
        if on:
            current_motor()[self.name] = str(1)
            print("{}=1".format(self.name))
        else:
            current_motor()[self.name] = str(0)
            print("{}=0".format(self.name))

class APIDir(APIBool):
    def __init__(self, *args, **kwargs):
        super(APIDir, self).__init__(*args, **kwargs)

    def update(self):
        val = 0.0
        try:
            val = float(current_motor()[self.name].get())
        except ValueError:
            pass
        status = val == 1.0 or val == 0.0
        self.checkbox.setChecked(status)

    def clicked(self, on):
        if on:
            current_motor()[self.name] = str(1)
            print("{}=1".format(self.name))
        else:
            current_motor()[self.name] = str(-1)
            print("{}=0".format(self.name)) 

class NumberEditSlider(NumberEdit):
    def __init__(self, *args, **kwargs):
        super(NumberEditSlider, self).__init__(*args, **kwargs)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.valueChanged.connect(self.valueChanged)
        self.number_widget.editingFinished.connect(self.valueChangedEdit)
        self.layout.addWidget(self.slider)
        #self.setLayout(layout)

    def valueChanged(self, value):
        self.setNumber(value)
        self.signal.emit(float(value))

    def valueChangedEdit(self):
        try:
            self.slider.setValue(int(float(self.number_widget.text())))
        except ValueError:
            pass

class ParameterEdit(NumberEdit):
    def __init__(self, *args, **kwargs):
        super(ParameterEdit, self).__init__(*args, **kwargs)
        self.signal.connect(self.set_value)

    def refresh_value(self):
        self.setNumber(current_motor()[self.name].get())

    def set_value(self, value):
        print("set {}={}".format(self.name, value))
        current_motor()[self.name] = str(value)

class StatusDisplay(QPushButton):
    def __init__(self, *args, **kwargs):
        super(StatusDisplay, self).__init__(*args, **kwargs)

    def mousePressEvent(self, event):
        pass

    def keyPressEvent(self, event):
        pass

class FaultDisplay(QWidget):
    def __init__(self, name, *args, **kwargs):
        super(FaultDisplay, self).__init__(*args, **kwargs)
        self.button = StatusDisplay(name, *args, **kwargs)
        self.name = name
        self.layout = QHBoxLayout()
        self.radio = QRadioButton(*args, **kwargs)
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.radio)
        self.radio.setFixedWidth(30)
        self.layout.setContentsMargins(QMargins(0,0,0,0))
        self.setLayout(self.layout)
        self.radio.clicked.connect(self.change_mask)

    def change_mask(self, event):
        mask = current_motor().error_mask()
        if self.name in mask.keys():
            if not mask[self.name]:
                mask[self.name] = True
            else:
                mask[self.name] = False
            current_motor().set_error_mask(mask)
    
    def update(self, faults, mask):
        disabled = False
        if self.button.text() in mask.keys():
            if not mask[self.button.text()]:
                disabled = True
            else:
                disabled = False
        if self.button.text() in faults.keys():
            if faults[self.button.text()]:
                if disabled:
                    self.button.setStyleSheet("background-color: orange")
                else:
                    self.button.setStyleSheet("background-color: red")
            else:
                self.button.setStyleSheet("background-color: green")
        if disabled:
            self.button.setDisabled(True)
            self.radio.setChecked(False)
        else:
            self.button.setDisabled(False)
            self.radio.setChecked(True)

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

Kc = np.matrix([[2.0/3, -1.0/3, -1.0/3], [0, 1.0/np.sqrt(3), -1.0/np.sqrt(3)]])

def calculate_vq(pos, va, vb, vc):
    cos_t = np.cos(pos)
    sin_t = np.sin(pos)

    valpha_beta = (Kc*np.asmatrix(np.block([va,vb,vc])).transpose()).transpose()*1.5
    vd = np.asarray(cos_t) * np.asarray(valpha_beta[:,0]) + np.asarray(-sin_t) * np.asarray(valpha_beta[:,1])
    vq = np.asarray(sin_t) * np.asarray(valpha_beta[:,0]) + np.asarray(cos_t) * np.asarray(valpha_beta[:,1])
    return (vd, vq)

def calculate_iq(pos, ia, ib, ic):
    cos_t = np.cos(pos)
    sin_t = np.sin(pos)

    ialpha_beta = (Kc*np.asmatrix(np.block([ia,ib,ic])).transpose()).transpose()
    id = np.asarray(cos_t) * np.asarray(ialpha_beta[:,0]) + np.asarray(-sin_t) * np.asarray(ialpha_beta[:,1])
    iq = np.asarray(sin_t) * np.asarray(ialpha_beta[:,0]) + np.asarray(cos_t) * np.asarray(ialpha_beta[:,1])
    return (id, iq)

class MotorTab(QWidget):
    def __init__(self, *args, **kwargs):
        super(MotorTab, self).__init__(*args, **kwargs)
        self.setAutoFillBackground(True)
        self.update_time = 100
        self.update_list = []

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)

    def update(self):
        global motor_manager
        #print("update: " + self.name)
        num_retries = 0
        while num_retries < 10:
            try:
                self.status = motor_manager.read()[0]
                for item in self.update_list:
                    item.update()
                break
            except RuntimeError:
                window.refresh()
                num_retries +=1
        if num_retries == 10:
            raise RuntimeError("Failed to reconnect")


    def pause(self):
        self.timer.stop()

    def unpause(self):
        self.timer.start(self.update_time)
    
def clear_fault_mask(text):
    print(text)


class FaultTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(FaultTab, self).__init__(*args, **kwargs)

        self.name = "fault"
        #print("init: " + self.name)
        mask = current_motor().error_mask()
        self.fields = ["mode", *list(mask.keys())]
        self.faults = []
        outer_layout = QHBoxLayout()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.setContentsMargins(QMargins(0,0,0,0))


        for field in self.fields:
            widget = FaultDisplay(field)
            if field in mask.keys():
                if not mask[field]:
                    widget.button.setDisabled(True)

            self.faults.append(widget)
            layout.addWidget(widget)

        layout.setSpacing(0)
        outer_layout.addLayout(layout)

        layout2 = QVBoxLayout()
        self.button = QPushButton("driver_enable")
        self.button.clicked.connect(self.driver_enable)
        layout2.addWidget(self.button)
        outer_layout.addLayout(layout2)
        self.setLayout(outer_layout)

    def driver_enable(self):
        print("driver enable")
        motor_manager.set_command_mode(motor.ModeDesired.DriverEnable)
        motor_manager.write_saved_commands()


    def update(self):
        super(FaultTab, self).update()

        mask = current_motor().error_mask()
        self.faults[0].button.setText("mode: " + self.status.flags.mode.name.lower())
        faults = self.status.flags.error.bits
        for widget in self.faults:
            widget.update(faults, mask)


class StatusTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(StatusTab, self).__init__(*args, **kwargs)
        self.setAutoFillBackground(True)

        self.name = "status"
        #print("init: " + self.name)
        #self.field_names = current_motor()["help"].get().split('\n')
        

        self.statuses = []
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        #layout.setContentsMargins(QMargins(0,0,0,0))
        for i in range(10):
            self.statuses.append(StatusCombo())
            self.statuses[i].layout.setContentsMargins(QMargins(0,0,0,0))
            layout.addWidget(self.statuses[i])
            self.statuses[i].signal.connect(self.valueEdit)
        self.set_field_options()
        self.statuses[0].combo_box.setCurrentText("vbus")
        self.statuses[1].combo_box.setCurrentText("5V")
        self.statuses[2].combo_box.setCurrentText("3v3")
        self.statuses[3].combo_box.setCurrentText("mraw")
        self.statuses[4].combo_box.setCurrentText("oraw")
        self.statuses[5].combo_box.setCurrentText("Tmotor")
        self.statuses[6].combo_box.setCurrentText("Tambient")
        self.statuses[7].combo_box.setCurrentText("Tboard")
        self.statuses[8].combo_box.setCurrentText("i5V")
        self.statuses[9].combo_box.setCurrentText("i48V")
        self.setLayout(layout)


    def valueEdit(self, s, v):
        print("value edit: {}={}".format(s,v))
        if v:
            current_motor()[s] = v

    def update(self):
        super(StatusTab, self).update()
        for status in self.statuses:
            if not status.text_widget.hasFocus():
                val = current_motor()[status.combo_box.currentText()].get()
                status.setText(val)


    def set_field_options(self):
        self.field_names = sorted(current_motor().get_api_options())
        for i in range(10):
            current_text = self.statuses[i].combo_box.currentText()
            self.statuses[i].combo_box.clear()
            self.statuses[i].combo_box.addItems(self.field_names)
            self.statuses[i].combo_box.setCurrentText(current_text)


class PlotTab(MotorTab):
    global motor_manager

    def __init__(self):
        super(PlotTab, self).__init__()

        self.name = "plot"
        self.update_time = 10

        self.chart = StreamingChart()
        self.layout = QVBoxLayout()
        self.combo_box = QComboBox()
        self.combo_box.addItems(["motor_position", "joint_position", "iq", "torque", "motor_encoder", 
            "motor_velocity", "joint_velocity", "iq_desired", "reserved", "encoder_error", "cogging"])
        self.combo_box.currentIndexChanged.connect(self.chart.removePoints)
        self.combo_box.currentIndexChanged.connect(self.name_change)
        self.layout.addWidget(self.combo_box)
        self.layout.addWidget(self.chart)
        var_layout = QHBoxLayout()
        self.pp = NumberDisplay("peak-peak")
        var_layout.addWidget(self.pp)
        self.std = NumberDisplay("std dev")
        var_layout.addWidget(self.std)
        self.mean = NumberDisplay("mean")
        var_layout.addWidget(self.mean)
        self.layout.addLayout(var_layout)
        self.setLayout(self.layout)


        s = motor_manager.read()[0]
        self.mcu_timestamp = s.mcu_timestamp
        self.t_seconds = 0.0

    def name_change(self):
        plot_str = self.combo_box.currentText()
        print(plot_str)
        if plot_str == "encoder_error":
            self.gear_ratio = 1
            try:
                self.gear_ratio = float(current_motor()["gear_ratio"].get())
            except ValueError:
                pass
            self.chart.length=10000
        else:
            self.chart.length=500

    def update(self):
        super(PlotTab, self).update()
        s = self.status
        self.t_seconds += (motor.diff_mcu_time(s.mcu_timestamp, self.mcu_timestamp))/cpu_frequency
        self.mcu_timestamp = s.mcu_timestamp
        plot_str = self.combo_box.currentText()
        val = 0
        x = self.t_seconds
        if plot_str == "encoder_error":
            val = s.motor_position - s.joint_position * self.gear_ratio
        elif plot_str == "cogging":
            val = s.iq
            x = np.mod(s.motor_position, 2*np.pi)
        else:
            val = getattr(s, plot_str)
        
        self.chart.update(x, [val])

        val = np.array([d.y() for d in self.chart.series[0].pointsVector()])
        self.std.setNumber(np.std(val))
        self.mean.setNumber(np.mean(val))
        self.pp.setNumber(max(val) - min(val))


class PlotTab2(MotorTab):
    global motor_manager

    def __init__(self):
        super(PlotTab2, self).__init__()

        self.name = "plot2"
        self.update_time = 10

        self.chart = StreamingChart()
        self.layout = QVBoxLayout()
        self.combo_box = QComboBox()
        self.set_field_options()
        self.layout.addWidget(self.combo_box)
        self.combo_box.setCurrentText("vbus")
        self.layout.addWidget(self.chart)
        var_layout = QHBoxLayout()
        self.pp = NumberDisplay("peak-peak")
        var_layout.addWidget(self.pp)
        self.std = NumberDisplay("std dev")
        var_layout.addWidget(self.std)
        self.mean = NumberDisplay("mean")
        var_layout.addWidget(self.mean)
        self.layout.addLayout(var_layout)
        self.setLayout(self.layout)
        self.setLayout(self.layout)

        self.combo_box.currentIndexChanged.connect(self.chart.removePoints)

        s = motor_manager.read()[0]
        self.mcu_timestamp = s.mcu_timestamp
        self.t_seconds = 0.0
      #  val = current_motor()[self.combo_box.currentText()].get()
      #  self.series.append(0, getattr(s, self.combo_box.currentText()))


    def update(self):
        super(PlotTab2, self).update()
        s = self.status
        self.t_seconds += (motor.diff_mcu_time(s.mcu_timestamp, self.mcu_timestamp))/cpu_frequency
        self.mcu_timestamp = s.mcu_timestamp
        val = current_motor()[self.combo_box.currentText()].get()
        try:
            self.chart.update(self.t_seconds, [float(val)])
            val2 = np.array([d.y() for d in self.chart.series[0].pointsVector()])
            self.std.setNumber(np.std(val2))
            self.mean.setNumber(np.mean(val2))
            self.pp.setNumber(max(val2) - min(val2))
        except ValueError:
            pass

    def set_field_options(self):
        self.field_names = sorted(current_motor().get_api_options())
        current_text = self.combo_box.currentText()
        self.combo_box.clear()
        self.combo_box.addItems(self.field_names)
        self.combo_box.setCurrentText(current_text)

class VelocityTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(VelocityTab, self).__init__(*args, **kwargs)

        self.update_time = 50

        self.name = "velocity"
        self.numbers = []
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.widget = NumberEditSlider("velocity")
        #self.widget.number_widget.editingFinished.connect(self.position_update)
        self.widget.slider.setMinimum(-1000)
        self.widget.slider.setMaximum(1000)
        self.widget.slider.setPageStep(50)
        self.widget.signal.connect(self.velocity_update)

        self.chart = QChart()
        self.chart_view = QChartView(self.chart)
        self.chart_view.setRubberBand(QChartView.VerticalRubberBand)
        self.series = QLineSeries()
        self.series.setUseOpenGL(True)
        self.chart.addSeries(self.series)
        self.axis_x = QValueAxis()
        self.axis_x.setTickCount(10)
        self.axis_x.setTitleText("Time (s)")
        self.chart.addAxis(self.axis_x, Qt.AlignmentFlag.AlignBottom)
        self.series.attachAxis(self.axis_x)
        self.axis_y = QValueAxis()
        self.axis_y.setTickCount(10)
        self.axis_y.setTitleText("Motor velocity (rad/s)")
        self.chart.addAxis(self.axis_y, Qt.AlignmentFlag.AlignLeft)
        self.series.attachAxis(self.axis_y)
        self.axis_y.setRange(-100,100)

        parameter_layout = QGridLayout()
        self.kp = ParameterEdit("vkp","kp")
        self.ki = ParameterEdit("vki","ki")
        self.max = ParameterEdit("vmax","current limit")
        self.max.signal.connect(lambda val: current_motor()["vki_limit"].set(str(val)))
        self.accel = ParameterEdit("vacceleration_limit","acceleration limit")
        self.imax = ParameterEdit("imax","voltage limit")
        self.imax.signal.connect(self.set_imax)
        self.filter = ParameterEdit("voutput_filt", "output filter")

        parameter_layout.addWidget(self.kp,0,0)
        parameter_layout.addWidget(self.ki,0,1)
        parameter_layout.addWidget(self.max,1,0)
        parameter_layout.addWidget(self.accel,1,1)
        parameter_layout.addWidget(self.imax,2,0)
        parameter_layout.addWidget(self.filter,2,1)

        layout.addWidget(self.widget)
        self.velocity_measured = NumberDisplay("velocity measured")
        layout.addWidget(self.chart_view)
        layout.addWidget(self.velocity_measured)
        layout.addLayout(parameter_layout)
        self.setLayout(layout)
        self.mcu_timestamp = 0
        self.motor_position = 0
        self.t_seconds = 0.0

    def update(self):
        super(VelocityTab, self).update()
        dt = (motor.diff_mcu_time(self.status.mcu_timestamp, self.mcu_timestamp))/cpu_frequency
        self.t_seconds += dt
        self.mcu_timestamp = self.status.mcu_timestamp
        dp = self.status.motor_position - self.motor_position
        self.motor_position = self.status.motor_position
        try:
            self.velocity_measured.setNumber(dp/dt)
        except ZeroDivisionError:
            pass

        vel = 0
        try:
            vel = dp/dt
        except ZeroDivisionError:
            pass
        
        if (abs(vel) < 10000):
            # reject rollovers
            try:
                self.series.append(self.t_seconds, vel)
                if len(self.series) > 200:
                    self.series.remove(0)
                self.axis_y.setMin(min([d.y() for d in self.series.pointsVector()]))
                self.axis_y.setMax(max([d.y() for d in self.series.pointsVector()]))
                self.axis_x.setMax(self.t_seconds)
                self.axis_x.setMin(self.series.at(0).x())
            except ValueError:
                pass

    def velocity_update(self):
        p = 0.0
        try:
            p = float(self.widget.number_widget.text())
        except ValueError:
            pass
        print("velocity command " + str(p))
        motor_manager.clear_commands()
        motor_manager.set_command_velocity([p])
        motor_manager.set_command_mode(motor.ModeDesired.Velocity)
        motor_manager.write_saved_commands()

    def set_imax(self, val):
        current_motor()["imax"].set(str(val))
        current_motor()["idmax"].set(str(val))
        current_motor()["iki_limit"].set(str(val))
        current_motor()["idki_limit"].set(str(val))

    def unpause(self):
        self.kp.refresh_value()
        self.ki.refresh_value()
        self.max.refresh_value()
        self.imax.refresh_value()
        self.accel.refresh_value()
        self.filter.refresh_value()
        return super().unpause()


class LogTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(LogTab, self).__init__(*args, **kwargs)

        self.name = "log"
        self.numbers = []

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.widget = QPlainTextEdit()

        layout.addWidget(self.widget)
        self.setLayout(layout)

    def update(self):
        super(LogTab, self).update()
        while True:
            log = current_motor()["log"].get()
            if log == "log end":
                break
            else:
                self.widget.appendPlainText(log)

class BringupTab(MotorTab):
    def __init__(self, main_window, *args, **kwargs):
        super(BringupTab, self).__init__(*args, **kwargs)

        self.name = "bringup"
        self.updating_enabled = True
        self.motor_handler = None
        self.package_info = None
        self.main_window = main_window

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.tc_files = []

        self.all_a_joints = ["left_ankle_x", "left_ankle_y", "left_shoulder_j1", "left_shoulder_j2", "left_elbow", "left_forearm_twist", 
                      "left_upper_arm_twist", "left_wrist_pitch", "left_wrist_yaw", "left_hip_x", "left_hip_y", "left_hip_z", 
                      "left_knee", "right_ankle_x", "right_ankle_y", "right_shoulder_j1", "right_shoulder_j2", "right_elbow", 
                      "right_forearm_twist", "right_upper_arm_twist", "right_wrist_pitch", "right_wrist_yaw", "right_hip_x", 
                      "right_hip_y", "right_hip_z", "right_knee", "spine_xy_left", "spine_xy_right", "spine_z"]
        self.all_b_joints = ["left_ankle_x", "left_ankle_y", "left_shoulder_j1", "left_shoulder_j2", "left_elbow", "left_forearm_twist", 
                      "left_upper_arm_twist", "left_wrist_pitch", "left_wrist_yaw", "left_hip_x", "left_hip_y", "left_hip_z", 
                      "left_knee", "right_ankle_x", "right_ankle_y", "right_shoulder_j1", "right_shoulder_j2", "right_elbow", 
                      "right_forearm_twist", "right_upper_arm_twist", "right_wrist_pitch", "right_wrist_yaw", "right_hip_x", 
                      "right_hip_y", "right_hip_z", "right_knee", "spine_x", "spine_z"]
        self.all_fingers = [
            'left_index_finger', 'left_middle_finger', 'left_ring_finger', 'left_pinky_finger',
            'left_thumb_abduct', 'left_thumb_flex', 'right_index_finger', 'right_middle_finger',
            'right_ring_finger', 'right_pinky_finger', 'right_thumb_abduct', 'right_thumb_flex'
        ]
        self.all_tcell_types = ["figure", "futek", "nmb"]

        select_params_layout = QHBoxLayout()
        # Create the dropdowns
        self.platform_type_dropdown = QComboBox(self)
        self.joint_name_dropdown = QComboBox(self)

        self.torque_cell_type_dropdown = QComboBox(self)
        self.result_label = QLabel('Firmware Config: ', self)

        self.platform_type_dropdown.addItems(['a_sample', 'b_test', 'hands', 'f100_spi', "f100_spi_ld", "f50_uart"])
        self.joint_name_dropdown.addItems(self.all_a_joints)
        self.torque_cell_type_dropdown.addItems(self.all_tcell_types)


        self.platform_directory_map = { "a_sample": "a_sample/actuator_parameters_idir",
                                        "b_test": "b_sample_test/actuator_parameters",
                                        "hands": "a_sample/palm",
                                        "f100_spi": "b_sample/actuator_parameters",
                                        "f100_spi_ld": "b_sample/actuator_parameters",
                                        "f50_uart": "b_sample/actuator_parameters"}

        self.update_firmware()
        # Open a file dialog to select files for upload
        platform_type = self.platform_type_dropdown.currentText()
        joint_name = self.joint_name_dropdown.currentText()

        #TODO: when we can figure out the correct path based on the joint name use this code
        self.base_config_path = f"{project_path}/tools/obot/{self.platform_directory_map[platform_type]}/{joint_name}.json"

        # Connect signals
        self.platform_type_dropdown.currentIndexChanged.connect(self.update_joint_and_tcell_dropdowns)
        self.joint_name_dropdown.currentIndexChanged.connect(self.update_firmware)
        self.torque_cell_type_dropdown.currentIndexChanged.connect(self.update_firmware)
        select_params_layout.addWidget(self.platform_type_dropdown)
        select_params_layout.addWidget(self.joint_name_dropdown)
        select_params_layout.addWidget(self.torque_cell_type_dropdown)
        layout.addLayout(select_params_layout)
        layout.addWidget(self.result_label)

        self.compile_opts_combo_box = QComboBox()
        compile_opts = ["", "-DBROKEN_MAX31875"]
        self.compile_opts_combo_box.addItems(compile_opts)
        select_params_layout.addWidget(self.compile_opts_combo_box)
        layout.addLayout(select_params_layout)

        buttons_layout = QHBoxLayout()
        self.create_and_save_btn = QPushButton("Create File and Save To Yaml Package")
        self.create_and_save_btn.clicked.connect(self.create_file_update_yaml)
        buttons_layout.addWidget(self.create_and_save_btn)

        self.flash_all_btn = QPushButton("Flash all")
        self.flash_all_btn.setToolTip("Flash all")
        self.flash_all_btn.setDisabled(True)

        self.commit_and_push_btn = QPushButton("Commit and push")
        self.commit_and_push_btn.clicked.connect(self.commit_and_push)
        self.commit_and_push_btn.setDisabled(True)

        self.flash_all_btn.clicked.connect(self.run_flash_all_routine)
        buttons_layout.addWidget(self.flash_all_btn)
        buttons_layout.addWidget(self.commit_and_push_btn)

        layout.addLayout(buttons_layout)

        self.setLayout(layout)

    def get_torque_cell_list(self):
        if len(self.tc_files) > 0:
            return self.tc_files
        if project_path is not None:
            s3 = S3Server("figure-robot-configs")
            paginator = s3.s3session.meta.client.get_paginator("list_objects_v2")

            # Paginate through objects in the bucket
            for page in paginator.paginate(Bucket="figure-robot-configs", Prefix="torque_cell_calibration_files"):
                if "Contents" in page:
                    for obj in page["Contents"]:
                        # Extract the filename
                        filename = os.path.basename(obj["Key"])
                        self.tc_files.append(filename)

            return self.tc_files
        else:
            print("Please export PROJECT_PATH variable")
            return []

    def update_joint_and_tcell_dropdowns(self):
        # Clear the current items in the dropdown
        self.joint_name_dropdown.clear()

        # Read the selected fields
        platform_type = self.platform_type_dropdown.currentText()
        joint_name = self.joint_name_dropdown.currentText()
        self.base_config_path = f"{project_path}/tools/obot/{self.platform_directory_map[platform_type]}/{joint_name}.json"

        # Update the joint name dropdown based on the platform type
        if platform_type == 'a_sample':
            self.joint_name_dropdown.addItems(self.all_a_joints)
            self.torque_cell_type_dropdown.clear()
            self.torque_cell_type_dropdown.addItems(["figure", "futek", "nmb"])
        elif platform_type == 'b_test' or platform_type == "f100_spi" or platform_type == "f100_spi_ld"  or platform_type == "f50_uart":
            self.joint_name_dropdown.addItems(self.all_b_joints)
            self.torque_cell_type_dropdown.clear()
            self.torque_cell_type_dropdown.addItems(["figure"])
        elif platform_type == 'hands':
            self.joint_name_dropdown.addItems(self.all_fingers)
            self.torque_cell_type_dropdown.clear()
            self.torque_cell_type_dropdown.addItems(["None"])

    def update_firmware(self):
        # Get the selected platform type
        platform_type = self.platform_type_dropdown.currentText()
        joint_name = self.joint_name_dropdown.currentText()
        selected_tcell_type = self.torque_cell_type_dropdown.currentText()
        self.base_config_path = f"{project_path}/tools/obot/{self.platform_directory_map[platform_type]}/{joint_name}.json"

        if platform_type == "a_sample":
            if "spine" in joint_name:
                self.fw_type = f"{platform_type}_{selected_tcell_type}_with_enc"
            elif "ankle_y" in joint_name:
                self.fw_type = f"{platform_type}_ankle_y"                 
            else:
                self.fw_type = f"{platform_type}_{selected_tcell_type}_without_enc"
        elif platform_type == "b_test":
            if "ankle_y" in joint_name:
                self.fw_type = f"{platform_type}_ld"
            elif ("ankle_x" in joint_name) or ("forearm_twist" in joint_name) or ("wrist" in joint_name):
                self.fw_type = f"{platform_type}_hd11-14"
            else:
                self.fw_type = f"{platform_type}"
        elif platform_type == "f100_spi" or platform_type == "f50_uart" or platform_type == "f100_spi_ld":
                self.fw_type = f"{platform_type}"            
        elif platform_type == "hands":
            self.fw_type = "hands"

        self.result_label.setText(f'Firmware Config: { self.fw_type}')

    def update(self):
        super(BringupTab, self).update()


    def checkout_main(self):
        try:
            subprocess.run(['git', 'checkout', 'main'], cwd=project_path, check=True)
        except subprocess.CalledProcessError as e:
            try:
                subprocess.run(['git', 'stash'], cwd=project_path, check=True)
                stashed = True
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to stash contents and checkout main branch: {e}")
                raise RuntimeError(e)
            try:
                subprocess.run(['git', 'checkout', 'main'], cwd=project_path, check=True)
            except subprocess.CalledProcessError as e:
                raise RuntimeError(e)
        return

    def commit_and_push(self):
        stashed=False
        platform_type = self.platform_type_dropdown.currentText()
        joint_name = self.joint_name_dropdown.currentText()
        # Checkout the main branch in the project_path repo
        try:
            self.checkout_main()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to checkout main branch: {e}")
            return           

        # Create a branch based on platform_name and joint_name
        branch_name = f'user/motor_gui_auto/{platform_type}/{joint_name}'
        try:
            subprocess.run(['git', 'checkout', '-B', branch_name], cwd=project_path, check=True)
            print(f"Check out branch {branch_name}")
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Error", f"Failed to create branch: {e}")
            return

        # Commit self.dest_file and self.robot_package
        try:
            print(f"Add {self.dest_file} and {self.robot_package} to the commit")
            subprocess.run(['git', 'add', self.dest_file, self.robot_package], cwd=project_path, check=True)
            print(f"Commit files")
            subprocess.run(['git', 'commit', '-m', 'Commit message'], cwd=project_path, check=True)
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Error", f"Failed to commit files: {e}")
            return

        # Push the branch
        try:
            subprocess.run(['git', 'push', 'origin', branch_name], cwd=project_path, check=True)
            print("Push branch")
            QMessageBox.information(self, "Success", "Branch created, committed, and pushed successfully!")
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Error", f"Failed to push branch: {e}")

        if stashed:
            try:
                subprocess.run(['git', 'stash', 'apply'], cwd=project_path, check=True)
            except subprocess.CalledProcessError as e:
                QMessageBox.critical(self, "Error", f"Failed to apply stashed changes: {e}")
                return

    def create_file_update_yaml(self):
        self.joint_to_type = {
            "left_hip_y": "hd25",
            "left_hip_x": "hd20",
            "left_hip_z": "hd20",
            "left_knee": "hd25",
            "left_ankle_y": "ld",
            "left_ankle_x": "hd14",
            "right_shoulder_j1": "hd17",
            "right_shoulder_j2": "hd17",
            "right_upper_arm_twist": "hd17",
            "right_elbow": "hd17",
            "right_forearm": "hd11",
            "left_shoulder_j1": "hd17",
            "left_shoulder_j2": "hd17",
            "left_upper_arm_twist": "hd17",
            "left_elbow": "hd17",
            "left_forearm": "hd11",
            "right_hip_y": "hd25",
            "right_hip_x": "hd20",
            "right_hip_z": "hd20",
            "right_knee": "hd25",
            "right_ankle_y": "ld",
            "right_ankle_x": "hd14",
            "spine_z": "hd20",
            "spine_x": "hd20",
            "neck_no": "hd11",
            "neck_yes": "hd11",
        }


        package_file_dialog = QFileDialog(self)
        package_file_dialog.setDirectory(f"{project_path}/tools/obot")
        package_file_dialog.setFileMode(QFileDialog.ExistingFile)
        
        # The selected file is the YAML package - the parent directory is the destination for the new config file
        self.robot_package, _ = package_file_dialog.getOpenFileName(self, "Open File", "")
        self.robot_directory = os.path.dirname(self.robot_package)
        
        # use generic tcell gain
        self.s3_server = S3Server("figure-robot-configs")
        joint_name = self.joint_name_dropdown.currentText()

        self.tcell_file = f"{self.joint_to_type[joint_name]}-generic.json"
        print(f"Using torque cell file: {self.tcell_file}")

        motor_sn = current_motor().serial_number()
        utils.link_torque_cell_to_motor(self.s3_server, f"motor_calibration_files/{motor_sn}_*", self.tcell_file)

        # Create a new JSON file for this actuator and save it in the robot directory
        self.create_file_and_save()

        # Add a reference to this file to the yaml package
        self.update_motor_handler()
        if(self.package_info is not None):
            try:
                self.update_yaml_file()
                QMessageBox.information(self, "Success", f"Data written to {self.robot_package}!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")

        self.flash_all_btn.setDisabled(False)

    def update_yaml_file(self):
        new_lines = []
        serial_number_to_match = current_motor().serial_number()
        line_added = False
        joint_name = self.joint_name_dropdown.currentText()

        with open(self.robot_package, 'r') as file:
            for line in file:
                if f"{serial_number_to_match}" in line or f"{joint_name}" in line:
                    # If a line with this SN or joint name exists replace the line with the new setup for that SN
                    new_lines.append(f' - [{self.package_info}]\n')
                    line_added = True
                else:
                    new_lines.append(line)
        if not line_added:
            # if the SN didn't exist in the file already then add the line to the bottom of the file
            new_lines.append(f'  - [{self.package_info}]\n')  # Replace line with package_info

        with open(self.robot_package, 'w') as file:
            file.writelines(new_lines)

    def create_file_and_save(self):
        platform_type = self.platform_type_dropdown.currentText()
        joint_name = self.joint_name_dropdown.currentText()
        self.base_config_path = f"{project_path}/tools/obot/{self.platform_directory_map[platform_type]}/{joint_name}.json"
        print(f"{self.base_config_path}")
        motor_sn = current_motor().serial_number()
        # TODO uncomment this line when we have `motor_driver_parameters` in the b_sample folder and remove the line below
        # motor_driver_sn_file = f"{project_path}/tools/obot/{platform_type}/motor_driver_parameters/{current_motor().serial_number()}.json"
        motor_driver_sn_file = f"{project_path}/tools/obot/a_sample/motor_driver_parameters/{motor_sn}.json"

        if not os.path.exists(motor_driver_sn_file):
            print(f"Cannot find {motor_driver_sn_file} so we're not including it")
            dictionary = {"inherits0": f"{os.path.relpath(self.base_config_path, self.robot_directory)}"}
        else:
            dictionary = {"inherits0": f"{os.path.relpath(self.base_config_path, self.robot_directory)}",
                        "inherits1": f"{os.path.relpath(motor_driver_sn_file, self.robot_directory)}"}
        
        # Create the file with the name given by the user
        self.dest_file = self.robot_directory + "/" + f"{joint_name}" + ".json"

        print(f"Creating a new configuration file at {self.dest_file}")
        with open(self.dest_file, "w") as file:
            json.dump(dictionary, file, indent=4)
        try:
            with open(self.dest_file, "w") as file:
                json.dump(dictionary, file, indent=4)
            QMessageBox.information(self, "Success", f"File saved to {self.dest_file}!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred while creating the file: {str(e)}")

    def update_motor_handler(self):
        self.board_type = "RNONE"
        match = re.search(r'([^/]+)\.json$', self.dest_file)
        self.motor_name = match.group(1)
        self.compile_opts = self.compile_opts_combo_box.currentText()
        self.package_info = f"\"{current_motor().serial_number()}\", \"{self.fw_type}\", \"{self.motor_name}\", \"\'{self.board_type}\'\""
        motor_info = {self.motor_name :{"fw_type": self.fw_type, "pcb_type": f"\'{self.board_type} {self.compile_opts}\'", "sn":current_motor().serial_number()}}
        try:
            self.motor_handler = MotorHandler( self.robot_directory,
                                                None,
                                                motor_info,
                                                self.motor_name,
                                                no_firmware_log = True
                                                )
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")

    def run_flash_params_routine(self):
        print("Flash param binary")
        self.update_motor_handler()

        # Disable updating while flashing the device since it will be nonresponsive in dfu mode
        self.updating_enabled = False

        try:
            self.motor_handler.run_flash_params_routine()
            time.sleep(6)
            QMessageBox.information(self, "Success", "The operation was successful!")
            # Wait for the device to show up again
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")


        # Reenable updates
        self.updating_enabled = True

    def run_flash_firmware_routine(self):
        print("Flash firmware")
        self.update_motor_handler()

        # Disable updating while flashing the device since it will be nonresponsive in dfu mode
        self.updating_enabled = False
        try:
            self.motor_handler.run_flash_firmware_routine()
            time.sleep(6)  
            QMessageBox.information(self, "Success", "The operation was successful!")
            # Wait for the device to show up again
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")
        # Reenable updates
        self.updating_enabled = True

    def run_flash_all_routine(self):
        print("Flash firmware and params")
        self.update_motor_handler()

        # Disable updating while flashing the device since it will be nonresponsive in dfu mode
        self.updating_enabled = False
        try:
            self.motor_handler.run_flash_all_routine()
            time.sleep(6)
            QMessageBox.information(self, "Success", "The operation was successful!")
            # Wait for the device to show up again
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")
        # Reenable updates
        self.updating_enabled = True
        self.commit_and_push_btn.setDisabled(False)


class CalibrateTab(MotorTab):
    def __init__(self, main_window, *args, **kwargs):
        super(CalibrateTab, self).__init__(*args, **kwargs)
        self.main_window = main_window

        self.name = "calibrate"
        self.numbers = []

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.electrical_zero_pos = APIDisplay("electrical_zero_pos")
        layout.addWidget(self.electrical_zero_pos)
        self.update_list.append(self.electrical_zero_pos)

        self.index_offset = APIDisplay("index_offset_measured")
        layout.addWidget(self.index_offset)
        self.update_list.append(self.index_offset)

        self.current_d = APIDisplay("id", "id measured (A)")
        layout.addWidget(self.current_d)
        self.update_list.append(self.current_d)

        self.phase_lock_current_startup = APIDisplay("startup_phase_lock_current", "startup phase lock current (A)", tooltip="startup_param.phase_lock_current")
        layout.addWidget(self.phase_lock_current_startup)
        self.update_list.append(self.phase_lock_current_startup)

        self.phase_lock_current = NumberEdit("phase lock current (A)", tooltip="command.current_desired")
        self.phase_lock_current.setNumber(current_motor()["startup_phase_lock_current"])
        layout.addWidget(self.phase_lock_current)

        mode_layout = QHBoxLayout()
        self.button = QPushButton("phase lock")
        self.button.setToolTip("command.mode_desired = PHASE_LOCK")
        self.button.clicked.connect(self.phase_lock)
        mode_layout.addWidget(self.button)

        self.obutton = QPushButton("open")
        self.obutton.setToolTip("command.mode_desired = OPEN")
        self.obutton.clicked.connect(mode_open)
        mode_layout.addWidget(self.obutton)

        self.jbutton = QPushButton("joint position")
        self.jbutton.setToolTip("command.mode_desired = JOINT_POSITION")
        self.jbutton.clicked.connect(self.joint_position)
        mode_layout.addWidget(self.jbutton)

        layout.addLayout(mode_layout)

        boollayout = QHBoxLayout()
        self.position_limits_disable = APIBool("disable_position_limits", "disable position limits")
        self.phase_mode = APIBool("phase_mode", "phase mode", "fast_loop_param.phase_mode")
        self.idir = APIBool("idir", "current direction", "fast_loop_param.current_direction")
        boollayout.addWidget(self.position_limits_disable)
        boollayout.addWidget(self.phase_mode)
        boollayout.addWidget(self.idir)
        self.update_list.append(self.position_limits_disable)
        self.update_list.append(self.phase_mode)
        self.update_list.append(self.idir)
        layout.addLayout(boollayout)
        
        self.obias = APIEdit("obias", "output bias (rad)", "main_loop_param.output_encoder.bias")
        self.obias.signal.connect(self.obias_set)
        self.oposition = NumberDisplay("output position (rad)", tooltip="status.joint_position")
        self.odir = APIDir("odir", "output position dir", "main_loop_param.output_encoder.dir")
        olayout = QHBoxLayout()
        olayout.addWidget(self.obias)
        olayout.addWidget(self.oposition)
        olayout.addWidget(self.odir)
        self.update_list.append(self.obias)
        self.update_list.append(self.odir)
        layout.addLayout(olayout)

        self.mbias = APIEdit("startup_mbias", "startup motor bias (rad)", "also calls set_startup_bias\nstartup_param.motor_encoder_bias")
        self.mbias.signal.connect(self.mbias_set)
        self.mposition = NumberDisplay("motor position (rad)", tooltip="status.motor_position")
        self.mdir = APIDir("mdir", "motor position dir", "fast_loop_param.motor_encoder.dir")

        mlayout = QHBoxLayout()
        mlayout.addWidget(self.mbias)
        mlayout.addWidget(self.mposition)
        mlayout.addWidget(self.mdir)
        self.update_list.append(self.mbias)
        self.update_list.append(self.mdir)
        layout.addLayout(mlayout)

        self.tbias = APIEdit("tbias", "torque bias (Nm)", "main_loop_param.torque_sensor.bias")
        self.torque = NumberDisplay("torque (Nm)", tooltip="status.torque")
        self.tdir = APIDir("tdir", "torque dir", "main_loop_param.torque_sensor.dir")
        tlayout = QHBoxLayout()
        tlayout.addWidget(self.tbias)
        tlayout.addWidget(self.torque)
        tlayout.addWidget(self.tdir)
        self.update_list.append(self.tbias)
        self.update_list.append(self.tdir)
        layout.addLayout(tlayout)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        find_limits_layout = QHBoxLayout()
        self.find_limits_velocity = NumberEdit("velocity", tooltip="command.velocity_desired")
        find_limits_layout.addWidget(self.find_limits_velocity)
        self.find_limits_current = NumberEdit("current", tooltip="command.current_desired")
        find_limits_layout.addWidget(self.find_limits_current)
        layout.addLayout(find_limits_layout)
        self.find_limits_button = QPushButton("find limits")
        self.find_limits_button.setToolTip("command.mode_desired = FIND_LIMITS")
        self.find_limits_button.clicked.connect(self.find_limits)
        layout.addWidget(self.find_limits_button)

        project_layout = QHBoxLayout()
        projects = []

        save_to_flash_layout = QHBoxLayout()
        self.set_robot_button = QPushButton("Select yaml package for the actuator")
        self.set_robot_button.clicked.connect(self.select_yaml_package)
        save_to_flash_layout.addWidget(self.set_robot_button)
        self.robot_label = QLineEdit(self)
        save_to_flash_layout.addWidget(self.robot_label)
        layout.addLayout(save_to_flash_layout)

        self.button = QPushButton("Save runtime values")
        self.button.setToolTip("Save runtime values")
        self.button.clicked.connect(self.run_read_runtime_and_save_to_flash_routine)
        save_to_flash_layout.addWidget(self.button)
        layout.addLayout(save_to_flash_layout)

        self.setLayout(layout)

    def read_yaml(self, filename: str) -> dict[str, dict[str, str]]:
        package_dict: dict[str, dict[str, str]] = {}
        with open(os.path.expanduser(filename), "r") as file:
            d = yaml.safe_load(file)
            for line in d["config"]:
                if line[0] == current_motor().serial_number():
                    package_info = {}
                # for example:
                # config:
                #  - ["207539635356", "motor_aksim", "J1", "R4"]
                sn = line[0]
                fw_type = line[1]
                name = line[2]
                pcb_type = line[3]
                package_dict[name] = {"sn": sn, "fw_type": fw_type, "pcb_type": pcb_type}
        return package_dict

    def select_yaml_package(self) -> None:
        # TODO: why does native Dialog not work here?
        # file_dialog = QFileDialog(self)
        # file_dialog.setDirectory(path)  # Set the current directory
        # self.device_config_path, _ = file_dialog.getOpenFileName(self, "Select File", "")
        # Open a file dialog to select files for upload
        path = f"{project_path}/tools/obot/"
        self.device_config_path, _ = QFileDialog.getOpenFileName(self, "Select File", path, options=QFileDialog.DontUseNativeDialog)
        print(self.main_window.get_ip_address())
        self.robot_config_path = os.path.dirname(self.device_config_path)
        print(f"Setting obot_config_path to: {self.robot_config_path}")
        self.robot_label.setText(self.robot_config_path)

    def update(self):
        super(CalibrateTab, self).update()
        self.oposition.setNumber(self.status.joint_position)
        self.mposition.setNumber(self.status.motor_position)
        self.torque.setNumber(self.status.torque)

    def phase_lock(self):
        print("phase lock")
        motor_manager.set_command_mode(motor.ModeDesired.PhaseLock)
        motor_manager.set_command_current([self.phase_lock_current.getNumber()])
        motor_manager.write_saved_commands()

    def joint_position(self):
        print("joint position")
        motor_manager.set_commands([motor.Command()])
        motor_manager.set_command_mode(motor.ModeDesired.JointPosition)
        motor_manager.write_saved_commands()

    def find_limits(self):
        print("find limits")
        motor_manager.set_command_mode(motor.ModeDesired.FindLimits)
        motor_manager.set_command_current([self.find_limits_current.getNumber()])
        motor_manager.set_command_velocity([self.find_limits_velocity.getNumber()])
        motor_manager.write_saved_commands()

    def mbias_set(self):
        print("setting mbias")
        current_motor()["set_startup_bias"].get()

    def obias_set(self):
        pass

    def run_read_runtime_and_save_to_flash_routine(self):
        motor_name = current_motor().name() #motors[0].name()
        motor_info = {}
        ip_address = self.main_window.get_ip_address()
        print(ip_address)
        with open(os.path.expanduser(self.device_config_path), "r") as file:
            data = yaml.safe_load(file)
            for line in data["config"]:
                if line[0] == current_motor().serial_number():
                    motor_info = {motor_name :{"fw_type": line[1], "pcb_type": line[3], "sn":current_motor().serial_number()}}
                    break

        self.motor_handler = MotorHandler( self.robot_config_path,
                                            None,
                                            motor_info,
                                            motor_name,
                                            no_firmware_log=True)
        self.updating_enabled = False
        # Disable updating while flashing the device since it will be nonresponsive in dfu mode
        try:
            self.motor_handler.run_read_runtime_and_save_to_flash_routine(flash = False, ip_address = ip_address)
            time.sleep(5)
            QMessageBox.information(self, "Success", "The operation was successful!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {str(e)}")

        # Reenable updates
        self.updating_enabled = True


class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        global motor_manager
        motor_manager = motor.MotorManager()
        self.simulated = False
        motors = motor_manager.get_connected_motors(connect=False)
        if "-simulated" in QCoreApplication.arguments():
            self.simulated = True
            motors = motor_manager.get_motors_by_name(["sim1", "sim2"], connect=False, allow_simulated = True)
        elif "-version" in QCoreApplication.arguments():
            try:
                # _MEIPASS from pyinstaller
                with open(sys._MEIPASS + "/buildnum") as f:
                    print("version: ", end="")
                    for line in f:
                        print(line)
            except:
                print("version: local version")
            sys.exit(0)

            
        if len(motors) == 0:
            motors = motor_manager.get_motors_by_name(["sim1"], connect=False, allow_simulated = True)
            self.simulated = True
        print(motors)
        self.menu_bar = QMenuBar(self)
        self.motor_menu = QMenu("&Motor")
        self.menu_bar.addMenu(self.motor_menu)
        self.motor_ip_menu = QMenu("Motor&IP")
        self.menu_bar.addMenu(self.motor_ip_menu)
        self.setMenuBar(self.menu_bar)
        actions = []
        for m in motors:
            actions.append(self.motor_menu.addAction(m.name()))
            print(m.name())
        self.motor_menu.triggered.connect(lambda action: self.connect_motor(action.text()))
        self.ip_address = None

        self.joint_to_ip_map = {
            "left_hip_y": "192.168.50.10",
            "left_hip_x": "192.168.50.11",
            "left_hip_z": "192.168.50.12",
            "left_knee": "192.168.50.13",
            "left_ankle_y": "192.168.50.14",
            "left_ankle_x": "192.168.50.15",
            "left_shoulder_j1": "192.168.50.30",
            "left_shoulder_j2": "192.168.50.31",
            "left_upper_arm_twist": "192.168.50.32",
            "left_elbow": "192.168.50.33",
            "left_wrist_roll": "192.168.50.34:7771",
            "left_wrist_pitch": "192.168.50.34:7772",
            "left_wrist_yaw": "192.168.50.34:7770",
            "right_hip_y": "192.168.50.40",
            "right_hip_x": "192.168.50.41",
            "right_hip_z": "192.168.50.42",
            "right_knee": "192.168.50.43",
            "right_ankle_y": "192.168.50.44",
            "right_ankle_x": "192.168.50.56",
            "right_shoulder_j1": "192.168.50.20",
            "right_shoulder_j2": "192.168.50.21",
            "right_upper_arm_twist": "192.168.50.22",
            "right_elbow": "192.168.50.23",
            "right_wrist_roll": "192.168.50.24:7771",
            "right_wrist_pitch": "192.168.50.24:7772",
            "right_wrist_yaw": "192.168.50.24:7770",
            "spine_z": "192.168.50.50",
            "spine_x": "192.168.50.51",
        }

        ips = ["Enter custom IP or joint_name",
                "192.168.50.200"]
        for ip in ips:
            self.motor_ip_menu.addAction(ip)
        self.motor_ip_menu.triggered.connect(lambda action: self.handle_menu_action(action.text()))

        if "-i" in QCoreApplication.arguments():
            self.ip_address = QCoreApplication.arguments()[QCoreApplication.arguments().index("-i") + 1]
            self.connect_motor_ip(self.ip_address)
        else:
            self.connect_motor(motors[0].name())

        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.West)

        self.tuning_tab = QTabWidget()
        self.tuning_tab.setTabPosition(QTabWidget.TabPosition.West)
        self.tuning_tab.unpause = None

        self.status_tab = StatusTab()
        self.plot2_tab = PlotTab2()
        self.tabs.addTab(FaultTab(), "fault")
        self.tabs.addTab(self.status_tab, "status")
        self.tabs.addTab(PlotTab(), "plot")
        self.tabs.addTab(self.plot2_tab, "plot2")
        self.tabs.addTab(VelocityTab(), "velocity")
        self.tabs.addTab(LogTab(), "log")
        #self.tabs.addTab(self.tuning_tab, "tuning")
        self.tabs.addTab(CalibrateTab(self), "calibrate")
        self.tabs.addTab(CurrentTuningTab(), "current tuning")
        self.tabs.addTab(PositionTuningTab(), "position tuning")
        self.tabs.addTab(StepperTab(), "stepper")
        self.tabs.addTab(BringupTab(self), "bringup")

        self.setCentralWidget(self.tabs)
        self.last_tab = self.tabs.currentWidget()
        self.last_tab.unpause()

        self.last_tuning_tab = self.tuning_tab.currentWidget()

        self.tabs.currentChanged.connect(self.new_tab)
        self.tuning_tab.currentChanged.connect(self.new_tuning_tab)
        if "-fullscreen" in QCoreApplication.arguments():
            self.showFullScreen()

    def handle_menu_action(self, text):
        # Check the text of the action to determine what to do
        if "Enter" in text:
            self.prompt_for_custom_ip()
        else:
            self.connect_motor_ip(text)

    def prompt_for_custom_ip(self):
        ip, ok = QInputDialog.getText(self, 'Enter Motor IP or Motor name', 'IP Address:')
        if ok and ip:
            self.connect_motor_ip(ip)

    def refresh(self):
        # Try to reconnect the motors upon a refresh request
        motor_manager = motor.MotorManager()
        self.simulated = False
        motors = None
        if "-simulated" in QCoreApplication.arguments():
            self.simulated = True
            motors = motor_manager.get_motors_by_name(["sim1", "sim2"], connect=False, allow_simulated = True)
        else:
            motors = motor_manager.get_connected_motors(connect=False)
        print(motors)
        self.menu_bar = QMenuBar(self)
        self.motor_menu = QMenu("&Motor")
        self.menu_bar.addMenu(self.motor_menu)
        self.setMenuBar(self.menu_bar)
        actions = []
        for m in motors:
            actions.append(self.motor_menu.addAction(m.name()))
            print(m.name())
        self.motor_menu.triggered.connect(lambda action: self.connect_motor(action.text()))
        self.connect_motor(motors[0].name())

    def get_ip_address(self):
        return self.ip_address

    def connect_motor_ip(self, text):
        global cpu_frequency
        ip = None
        if is_ip_address(text):
            ip = text
        else:
            if text in self.joint_to_ip_map.keys():
                ip = self.joint_to_ip_map[text]
            else:
                raise RuntimeError(f"IP address for {text} is not defined")

        print("Connecting motor " + ip)
        self.ip_address = ip
        motor_manager.get_motors_by_ip([ip], allow_simulated = self.simulated)
        self.connect_motor_generic(ip)

    def connect_motor(self, name):
         global cpu_frequency
         print("Connecting motor " + name)
         motor_manager.get_motors_by_name([name], allow_simulated = self.simulated)
         self.connect_motor_generic(name)

    def connect_motor_generic(self, name):
        motor_manager.set_auto_count()
        current_motor()["api_timeout"] = "100000"
        current_motor().set_timeout_ms(500)
        self.setWindowTitle(name + " sn:" + current_motor().serial_number())
        cpu_frequency = current_motor().get_cpu_frequency()
        if hasattr(self, "status_tab"):
            self.status_tab.set_field_options()
            self.plot2_tab.set_field_options()

    def new_tab(self, index):
        #print("last tab " + str(index) + " " + self.last_tab.name)
        self.last_tab.pause()
        #print("new tab " + str(index) + " " + self.tabs.widget(index).name)
        self.tabs.widget(index).unpause()
        self.last_tab = self.tabs.widget(index)

    def new_tuning_tab(self, index):
        pass
       # self.last_tuning_tab.pause()
        #self.tuning_tab.widget(index).unpause()
        #self.last_tuning_tab = self.tuning_tab.widget(index)


class BodeWindow(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()
        self.label = QLabel("Bode plot")
        layout.addWidget(self.label)
        self.mag_chart = self.Chart()
        layout.addWidget(self.mag_chart.chart_view)
        self.phase_chart = self.Chart()
        self.phase_chart.axis_y.setTitleText("phase (deg)")
        layout.addWidget(self.phase_chart.chart_view)
        self.setLayout(layout)
        self.set_frequency_limits()
        self.set_magnitude_limits()
        self.set_phase_limits()

    def append(self, frequency, magnitude, phase):
        if frequency > 0:
            data = QPointF(frequency, 20*np.log10(magnitude))
            data_phase = QPointF(frequency, phase)
            self.mag_chart.series.append(data)
            self.phase_chart.series.append(data_phase)
    
    def set_frequency_limits(self, range_min = 100, range_max = 10000):
        self.mag_chart.axis_x.setRange(range_min, range_max)
        self.phase_chart.axis_x.setRange(range_min, range_max)

    def set_magnitude_limits(self, range_min = -20, range_max = 10):
        self.mag_chart.axis_y.setRange(range_min, range_max)

    def set_phase_limits(self, range_min = -200, range_max = 0):
        self.phase_chart.axis_y.setRange(range_min, range_max)


    class Chart(QWidget):
        def __init__(self):
            super().__init__()
            self.chart = QChart()
            self.chart_view = QChartView(self.chart)
            self.chart_view.setRubberBand(QChartView.VerticalRubberBand)
            self.series = QLineSeries()
            self.series.setUseOpenGL(True)
            self.chart.addSeries(self.series)
            self.axis_x = QLogValueAxis()
            self.axis_x.setMinorTickCount(10)
            self.axis_x.setTitleText("frequency (Hz)")
            self.chart.addAxis(self.axis_x, Qt.AlignmentFlag.AlignBottom)
            self.series.attachAxis(self.axis_x)
            self.axis_y = QValueAxis()
            self.axis_y.setTickCount(10)
            self.axis_y.setTitleText("magnitude (20*log10(mag))")
            self.chart.addAxis(self.axis_y, Qt.AlignmentFlag.AlignLeft)
            self.series.attachAxis(self.axis_y)

            self.chart_view.setMouseTracking(True)
            self.chart_view.viewport().installEventFilter(self)

        def eventFilter(self, obj, event):
            if obj is self.chart_view.viewport() and event.type() == QEvent.MouseMove:
                lp = event.pos()
                sp = self.chart_view.mapToScene(lp)
                vp = self.chart_view.chart().mapToValue(sp)
                print(vp)
            return super().eventFilter(obj, event)
            
            

class CurrentTuningTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(CurrentTuningTab, self).__init__(*args, **kwargs)

        self.update_time = 100
        self.name = "current_tuning"
        self.command = motor.Command()
        self.command.mode_desired = motor.ModeDesired.CurrentTuning
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.amplitude = NumberEditSlider("amplitude (A)")
        self.amplitude.slider.setMinimum(-100)
        self.amplitude.slider.setMaximum(100)
        self.amplitude.slider.setPageStep(5)
        self.amplitude.signal.connect(self.amplitude_update)
        layout.addWidget(self.amplitude)

        self.bias = NumberEditSlider("bias (A)")
        self.bias.slider.setMinimum(-100)
        self.bias.slider.setMaximum(100)
        self.bias.slider.setPageStep(5)
        self.bias.signal.connect(self.bias_update)
        layout.addWidget(self.bias)

        self.frequency = NumberEditSlider("frequency (Hz)")
        self.frequency.slider.setMinimum(0)
        self.frequency.slider.setMaximum(10000)
        self.frequency.slider.setPageStep(50)
        self.frequency.signal.connect(self.frequency_update)
        layout.addWidget(self.frequency)

        self.mode = QComboBox()
        self.mode.addItems(motor.TuningMode.__members__)
        self.mode.currentTextChanged.connect(self.mode_update)
        layout.addWidget(self.mode)

        parameter_layout = QGridLayout()
        self.kp = ParameterEdit("ikp", "kp (V/A)")
        self.kp.signal.connect(lambda val: [current_motor()["idkp"].set(str(val)), current_motor()["ikp2"].set(str(val)), current_motor()["idkp2"].set(str(val))])
        self.ki = ParameterEdit("iki", "ki (V/(A*T))")
        self.ki.signal.connect(lambda val: [current_motor()["idki"].set(str(val)), current_motor()["iki2"].set(str(val)), current_motor()["idki2"].set(str(val))])
        self.ki_limit = ParameterEdit("iki_limit", "ki_limit (V)")
        self.ki_limit.signal.connect(lambda val: current_motor()["idki_limit"].set(str(val)))
        self.command_max = ParameterEdit("imax", "command_max (V)")
        self.command_max.signal.connect(lambda val: current_motor()["idmax"].set(str(val)))
        self.filter = ParameterEdit("iq_filter", "current_filter (Hz)")
        self.filter.signal.connect(lambda val: current_motor()["id_filter"].set(str(val)))
        self.pwm_mult = ParameterEdit("pwm_mult")
        self.ilimit = ParameterEdit("ilimit", "current rate limit (A/s)", tooltip="fast_loop_param.foc_param.i{q,d}_rate_limit")
        self.ilimit.signal.connect(lambda val: current_motor()["ilimit"].set(str(val)))
        parameter_layout.addWidget(self.kp,0,0)
        parameter_layout.addWidget(self.ki,0,1)
        parameter_layout.addWidget(self.ki_limit,0,2)
        parameter_layout.addWidget(self.command_max,1,0)
        parameter_layout.addWidget(self.filter,1,1)
        parameter_layout.addWidget(self.pwm_mult,1,2)
        parameter_layout.addWidget(self.ilimit,2,0)
        layout.addLayout(parameter_layout)

        # self.text = QPlainTextEdit()
        # layout.addWidget(self.text)

        self.chart = QChart()
        self.chart_view = QChartView(self.chart)
        self.chart_view.setRubberBand(QChartView.VerticalRubberBand)
        self.series = QLineSeries()
        self.series.setUseOpenGL(True)
        self.series.setName("desired")
        self.series2 = QLineSeries()
        self.series2.setUseOpenGL(True)
        self.series2.setName("measured (filt)")
        self.chart.addSeries(self.series)
        self.chart.addSeries(self.series2)
        self.axis_x = QValueAxis()
        self.axis_x.setTickCount(10)
        self.axis_x.setTitleText("time (ms)")
        self.chart.addAxis(self.axis_x, Qt.AlignmentFlag.AlignBottom)
        self.series.attachAxis(self.axis_x)
        self.series2.attachAxis(self.axis_x)
        self.axis_y = QValueAxis()
        self.axis_y.setTickCount(10)
        self.axis_y.setTitleText("current (A)")
        self.chart.addAxis(self.axis_y, Qt.AlignmentFlag.AlignLeft)
        self.series.attachAxis(self.axis_y)
        self.series2.attachAxis(self.axis_y)
        self.axis_y.setRange(-100,100)
        self.axis_y2 = QValueAxis()
        self.axis_y2.setTickCount(10)
        self.axis_y2.setTitleText("voltage (V)")
        self.chart.addAxis(self.axis_y2, Qt.AlignmentFlag.AlignRight)
        self.seriesvd = QLineSeries()
        self.seriesvd.setUseOpenGL(True)
        self.seriesvd.setName("vd")
        self.chart.addSeries(self.seriesvd)
        self.seriesvd.attachAxis(self.axis_x)
        self.seriesvd.attachAxis(self.axis_y2)
        self.seriesvq = QLineSeries()
        self.seriesvq.setUseOpenGL(True)
        self.seriesvq.setName("vq")
        self.chart.addSeries(self.seriesvq)
        self.seriesvq.attachAxis(self.axis_x)
        self.seriesvq.attachAxis(self.axis_y2)

        layout.addWidget(self.chart_view)

        measure_layout = QGridLayout()
        self.freq_des = NumberDisplay("frequency desired")
        measure_layout.addWidget(self.freq_des, 0, 0)
        self.freq_meas = NumberDisplay("frequency measured")
        measure_layout.addWidget(self.freq_meas, 0, 1)
        self.phase = NumberDisplay("phase shift")
        measure_layout.addWidget(self.phase, 1, 0)
        self.mag_des = NumberDisplay("magnitude des")
        measure_layout.addWidget(self.mag_des, 1, 1)
        self.mag = NumberDisplay("magnitude")
        measure_layout.addWidget(self.mag, 1, 2)
        layout.addLayout(measure_layout)
        self.setLayout(layout)
        #t = np.matrix(np.linspace(0,94*(1/50000),95))
        self.freq_all =np.matrix(np.linspace(0,10000,10000//5+1)).transpose()
        #self.ei = np.exp(1j*self.freq*t*2*np.pi)

    def update(self):
        super(CurrentTuningTab, self).update()
        fast_log = current_motor().get_fast_log()
        #self.text.setPlainText(fast_log)

        try:
            num_poles = float(current_motor()["num_poles"].get())
            data = np.genfromtxt(StringIO(fast_log), delimiter=",", names=True, skip_footer=1, skip_header=0)
            inds = data["timestamp"].argsort()
            data = data[inds]
            t_seconds = data["timestamp"]/cpu_frequency*1000
            t_seconds -= max(t_seconds)


            iq_des = np.matrix(data["iq_des"]).transpose()
            iq_meas_filt = np.matrix(data["iq_meas_filt"]).transpose()
            va = np.matrix(data["va"]).transpose()
            vb = np.matrix(data["vb"]).transpose()
            vc = np.matrix(data["vc"]).transpose()
            pos = num_poles*np.matrix(data["position"]).transpose()
            dt = (t_seconds[1] - t_seconds[0])/1000

            cos_t = np.cos(pos)
            sin_t = np.sin(pos)

            Kc = np.matrix([[2.0/3, -1.0/3, -1.0/3], [0, 1.0/np.sqrt(3), -1.0/np.sqrt(3)]])
            valpha_beta = (Kc*np.block([va,vb,vc]).transpose()).transpose()*1.5
            vd = np.asarray(cos_t) * np.asarray(valpha_beta[:,0]) + np.asarray(-sin_t) * np.asarray(valpha_beta[:,1])
            vq = np.asarray(sin_t) * np.asarray(valpha_beta[:,0]) + np.asarray(cos_t) * np.asarray(valpha_beta[:,1])

            #fmeas = fft(iq_des)
            if self.command.current_tuning.mode == motor.TuningMode.Chirp:
                self.freq = self.freq_all
            else:
                self.freq = np.matrix([self.command.current_tuning.frequency])

            self.ei = np.exp(1j*self.freq*t_seconds/1000*2*np.pi)
            fmeas = self.ei*iq_des
            
            fmeasi = np.argmax(abs(fmeas))
            #freq = fftfreq(iq_des.size, dt)
            self.freq_des.setNumber(self.freq[fmeasi])
            mag_des = np.abs(fmeas[fmeasi])/iq_des.size*2
            self.mag_des.setNumber(mag_des)
            phas_des = np.angle(fmeas[fmeasi])

            fmeas_meas = self.ei*iq_meas_filt
            fmeas_mi = np.argmax(abs(fmeas_meas))
            self.freq_meas.setNumber(self.freq[fmeas_mi])
            mag_meas = np.abs(fmeas_meas[fmeasi])/iq_des.size*2
            phas_meas = np.angle(fmeas_meas[fmeasi])

            phase_deg = -(phas_meas - phas_des)*180/np.pi
            if phase_deg > 180:
                phase_deg -= 360
            self.phase.setNumber(phase_deg)
            self.mag.setNumber(mag_meas/mag_des)

            if self.command.current_tuning.mode == motor.TuningMode.Chirp:
                # plot bode
                self.bode_window.append(self.freq[fmeas_mi], np.abs(mag_meas/mag_des), phase_deg)


         
            # plot timeseries
            xy = [QPointF(x[0],x[1]) for x in np.column_stack((t_seconds, np.array(iq_des)))]
            self.series.replace(xy) #append(self.t_seconds, float(val))
            xy2 = [QPointF(x[0],x[1]) for x in np.column_stack((t_seconds, np.array(iq_meas_filt)))] # abs(fmeas)))]#iq_meas_filt))]
            self.series2.replace(xy2)
            xy3 = [QPointF(x[0],x[1]) for x in np.column_stack((t_seconds, np.array(vd)))]
            self.seriesvd.replace(xy3)
            xy4 = [QPointF(x[0],x[1]) for x in np.column_stack((t_seconds, np.array(vq)))]
            self.seriesvq.replace(xy4)
            min_y = min(min([d.y() for d in self.series.pointsVector()]), min([d.y() for d in self.series2.pointsVector()]))
            max_y = max(max([d.y() for d in self.series.pointsVector()]), max([d.y() for d in self.series2.pointsVector()]))
            self.axis_y.setMin(min_y)
            self.axis_y.setMax(max_y)
            self.axis_x.setMin(min(t_seconds))
            self.axis_x.setMax(max(t_seconds))
            min_y2 = min(min([d.y() for d in self.seriesvd.pointsVector()]), min([d.y() for d in self.seriesvq.pointsVector()]))
            max_y2 = max(max([d.y() for d in self.seriesvd.pointsVector()]), max([d.y() for d in self.seriesvq.pointsVector()]))
            self.axis_y2.setMin(min_y2)
            self.axis_y2.setMax(max_y2)
        except ValueError as err:
            print(traceback.format_exc())


    def unpause(self):
        self.kp.refresh_value()
        self.ki.refresh_value()
        self.ki_limit.refresh_value()
        self.command_max.refresh_value()
        self.filter.refresh_value()
        self.pwm_mult.refresh_value()
        return super().unpause()

    def command_update(self):
        print(self.command)
        motor_manager.write([self.command])

    def amplitude_update(self):
        try:
            self.command.current_tuning.amplitude = float(self.amplitude.number_widget.text())
            self.command_update()
        except ValueError:
            pass

    def bias_update(self):
        try:
            self.command.current_tuning.bias = float(self.bias.number_widget.text())
            self.command_update()
        except ValueError:
            pass

    def frequency_update(self):
        try:
            self.command.current_tuning.frequency = float(self.frequency.number_widget.text())
            self.command_update()
        except ValueError:
            pass

    def mode_update(self, selection):
        try:
            self.command.current_tuning.mode = int(motor.TuningMode.__members__[selection])
            if self.command.current_tuning.mode == motor.TuningMode.Chirp:
                self.bode_window = BodeWindow()
                self.bode_window.show()
            self.command_update()
        except ValueError:
            pass


class StreamingChart(QChartView):
    def __init__(self, num_lines=1, linetype=QLineSeries, *args, **kwargs):
        self.chart = QChart()
        super(StreamingChart, self).__init__(self.chart, *args, **kwargs)
        self.setRubberBand(QChartView.VerticalRubberBand)
        self.num_lines = num_lines
        self.length = 500
        
        self.axis_x = QValueAxis()
        self.axis_x.setTickCount(10)
        self.axis_x.setTitleText("Time (s)")
        self.chart.addAxis(self.axis_x, Qt.AlignmentFlag.AlignBottom)
        
        self.axis_y = QValueAxis()
        self.axis_y.setTickCount(10)        
        self.chart.addAxis(self.axis_y, Qt.AlignmentFlag.AlignLeft)
        
        self.axis_y.setRange(-100,100)
        #self.axis_y.setTitleText("Motor velocity (rad/s)")
        self.series = [1] * num_lines
        for i in range(num_lines):
            self.series[i] = linetype()
            self.series[i].setUseOpenGL(True)
            self.chart.addSeries(self.series[i])
            self.series[i].attachAxis(self.axis_x)
            self.series[i].attachAxis(self.axis_y)

    def update(self, t, data):
        try:
            min1 = float("inf")
            max1 = float("-inf")
            for i in range(self.num_lines):
                self.series[i].append(t, data[i])
                if len(self.series[i]) > self.length:
                    self.series[i].remove(0)
                min1 = min(min1,min([d.y() for d in self.series[i].pointsVector()]))
                max1 = max(max1,max([d.y() for d in self.series[i].pointsVector()]))

            self.axis_y.setMin(min1)
            self.axis_y.setMax(max1)
            self.axis_x.setMax(max([d.x() for d in self.series[0].pointsVector()]))
            self.axis_x.setMin(min([d.x() for d in self.series[0].pointsVector()]))
        except ValueError:
            pass
    
    def removePoints(self):
        for i in range(self.num_lines):
            self.series[i].removePoints(0,self.series[i].count())

class PositionTuningTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(PositionTuningTab, self).__init__(*args, **kwargs)
        self.update_time = 3

        self.name = "position_tuning"
        self.command = motor.Command()
        self.command.mode_desired = motor.ModeDesired.PositionTuning
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.amplitude = NumberEditSlider("amplitude (rad)")
        self.amplitude.slider.setMinimum(-100)
        self.amplitude.slider.setMaximum(100)
        self.amplitude.slider.setPageStep(5)
        self.amplitude.signal.connect(self.amplitude_update)
        layout.addWidget(self.amplitude)

        self.bias = NumberEditSlider("bias (rad)")
        self.bias.slider.setMinimum(-100)
        self.bias.slider.setMaximum(100)
        self.bias.slider.setPageStep(5)
        self.bias.signal.connect(self.bias_update)
        layout.addWidget(self.bias)

        self.frequency = NumberEditSlider("frequency (Hz)")
        self.frequency.slider.setMinimum(0)
        self.frequency.slider.setMaximum(100)
        self.frequency.slider.setPageStep(5)
        self.frequency.signal.connect(self.frequency_update)
        layout.addWidget(self.frequency)

        self.mode = QComboBox()
        self.mode.addItems(motor.TuningMode.__members__)
        self.mode.currentTextChanged.connect(self.mode_update)
        layout.addWidget(self.mode)

        parameter_layout = QGridLayout()
        self.kp = APIEdit("kp", "kp (A/rad)", "main_loop_param.position_controller.position.kp")
        self.kd = APIEdit("kd", "kd (A*s/rad)")
        #self.ki_limit = ParameterEdit("ki_limit")
        self.command_max = APIEdit("max", "command max (A)")
        self.output_filter = APIEdit("output_filter", "output filter (Hz)")
        self.velocity_filter = APIEdit("velocity_filter", "velocity_filter (Hz)")
        parameter_layout.addWidget(self.kp,0,0)
        parameter_layout.addWidget(self.kd,0,1)
        #parameter_layout.addWidget(self.ki_limit,1,0)
        parameter_layout.addWidget(self.command_max,1,0)
        parameter_layout.addWidget(self.output_filter,1,1)
        parameter_layout.addWidget(self.velocity_filter,2,0)
        layout.addLayout(parameter_layout)
        self.chart = StreamingChart(2)
        layout.addWidget(self.chart)
        self.chart2 = StreamingChart(1)
        layout.addWidget(self.chart2)

        self.setLayout(layout)
        self.mcu_timestamp = 0
        self.t_seconds = 0
        self.chart.series[0].setName("measured")
        self.chart.series[1].setName("desired")
        self.chart.axis_y.setTitleText("Motor position (rad)")
        self.chart2.series[0].setName("error")
        self.chart2.axis_y.setTitleText("Motor position (rad)")

    def update(self):
        super(PositionTuningTab, self).update()
        
        dt = (motor.diff_mcu_time(self.status.mcu_timestamp, self.mcu_timestamp))/cpu_frequency
        self.t_seconds += dt
        self.mcu_timestamp = self.status.mcu_timestamp

        error = 0.0
        try:
            error = float(current_motor()["error"].get())
        except ValueError:
            pass
        desired = self.status.motor_position + error

        self.chart.update(self.t_seconds, [self.status.motor_position, desired])
        self.chart2.update(self.t_seconds, [error])
  

    def unpause(self):
        self.kp.update()
        self.kd.update()
        self.command_max.update()
        self.output_filter.update()
        self.velocity_filter.update()
        return super().unpause()

    def command_update(self):
        print(self.command)
        motor_manager.write([self.command])

    def amplitude_update(self):
        try:
            self.command.position_tuning.amplitude = float(self.amplitude.number_widget.text())
            print("amplitude: {}".format(self.command.current_tuning.amplitude))
            self.command_update()
        except ValueError:
            pass

    def bias_update(self):
        try:
            self.command.position_tuning.bias = float(self.bias.number_widget.text())
            self.command_update()
        except ValueError:
            pass

    def frequency_update(self):
        try:
            self.command.position_tuning.frequency = float(self.frequency.number_widget.text())
            self.command_update()
        except ValueError:
            pass

    def mode_update(self, selection):
        try:
            self.command.position_tuning.mode = int(motor.TuningMode.__members__[selection])
            if self.command.position_tuning.mode == motor.TuningMode.Chirp:
                self.bode_window = BodeWindow()
                self.bode_window.set_frequency_limits(1, 500)
                self.bode_window.set_magnitude_limits(-40,10)
                self.bode_window.show()
            self.command_update()
        except ValueError:
            pass

class StepperTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(StepperTab, self).__init__(*args, **kwargs)
        self.update_time = 10

        self.name = "stepper_velocity"
        self.current = 0.
        self.velocity = 0.
        num_poles = current_motor()["num_poles"].get()
        try:
            self.num_poles = float(num_poles)
        except ValueError:
            self.num_poles = 1
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.current_slider = NumberEditSlider("current (A)")
        self.current_slider.slider.setMinimum(0)
        self.current_slider.slider.setMaximum(20)
        self.current_slider.slider.setPageStep(1)
        self.current_slider.signal.connect(self.current_update)
        layout.addWidget(self.current_slider)

        self.velocity_slider = NumberEditSlider("velocity (rad/s)")
        self.velocity_slider.slider.setMinimum(-200)
        self.velocity_slider.slider.setMaximum(200)
        self.velocity_slider.slider.setPageStep(5)
        self.velocity_slider.signal.connect(self.velocity_update)
        layout.addWidget(self.velocity_slider)

        self.num_points = NumberEdit("Number of plot points")
        self.num_points.signal.connect(self.num_points_update)
        layout.addWidget(self.num_points)
        

        self.current_chart = StreamingChart(5, QScatterSeries)
        layout.addWidget(self.current_chart)
        self.voltage_chart = StreamingChart(5, QScatterSeries)
        layout.addWidget(self.voltage_chart)
        
        self.num_points.setNumber(self.current_chart.length)

        self.setLayout(layout)
        self.mcu_timestamp = 0
        self.t_seconds = 0
        self.current_chart.series[0].setName("a")
        self.current_chart.series[1].setName("b")
        self.current_chart.series[2].setName("c")
        self.current_chart.series[3].setName("d")
        self.current_chart.series[4].setName("q")
        self.current_chart.axis_y.setTitleText("current (A)")
        self.voltage_chart.series[0].setName("a")
        self.voltage_chart.series[1].setName("b")
        self.voltage_chart.series[2].setName("c")
        self.voltage_chart.series[3].setName("d")
        self.voltage_chart.series[4].setName("q")
        self.voltage_chart.axis_y.setTitleText("voltage (V)")

        for series in self.current_chart.series + self.voltage_chart.series:
            series.setMarkerSize(5)

    def update(self):
        super(StepperTab, self).update()
        
        dt = (motor.diff_mcu_time(self.status.mcu_timestamp, self.mcu_timestamp))/cpu_frequency
        self.t_seconds += dt
        self.mcu_timestamp = self.status.mcu_timestamp

        status = current_motor()["fast_loop_status"].get()
        data = np.genfromtxt(StringIO(status), delimiter=",")
        data = data.copy()
        data.resize(10)
        motor_pos = data[1]
        iq_des = data[2]
        iq_meas = data[3]
        ia = data[4]
        ib = data[5]
        ic = data[6]
        va = data[7]
        vb = data[8]
        vc = data[9]

        (vd, vq) = calculate_vq(motor_pos*self.num_poles, va, vb, vc)
        (id, iq) = calculate_iq(motor_pos*self.num_poles, ia, ib, ic)

        x = np.mod(motor_pos,2*np.pi/self.num_poles)
        self.current_chart.update(x, [ia, ib, ic, id, iq_meas])
        self.voltage_chart.update(x, [va, vb, vc, vd, vq])
  


    def command_update(self):
        motor_manager.set_command_stepper_velocity(current=self.current, velocity=self.velocity)
        motor_manager.write_saved_commands()

    def current_update(self):
        try:
            self.current = float(self.current_slider.number_widget.text())
            print("current: {}".format(self.current))
            self.command_update()
            self.current_chart.removePoints()
            self.voltage_chart.removePoints()
        except ValueError:
            pass

    def velocity_update(self):
        try:
            self.velocity = float(self.velocity_slider.number_widget.text())
            self.command_update()
        except ValueError:
            pass

    def num_points_update(self, value):
        try:
            self.current_chart.length = int(value)
            self.voltage_chart.length = int(value)
        except ValueError:
            pass
        self.current_chart.removePoints()
        self.voltage_chart.removePoints()


app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()
