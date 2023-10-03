#!/usr/bin/env python3

import sys
import traceback

from PyQt5.QtCore import QTimer, Qt, QMargins, QCoreApplication, pyqtSignal, QPointF
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
    QCheckBox,
    QMenu,
    QPlainTextEdit,
    QFrame,
    QRadioButton
)
from PyQt5.QtGui import QPalette, QColor, QDoubleValidator
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
import motor
import numpy as np
from io import StringIO

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

motor_manager = None

def current_motor():
    return motor_manager.motors()[0]

cpu_frequency = 170e6

def mode_open():
    print("open")
    motor_manager.set_command_mode(motor.ModeDesired.Open)
    motor_manager.write_saved_commands()

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
        return float(self.number_widget.text())

    def editingFinished(self):
        self.signal.emit(float(self.number_widget.text()))

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
        
    def editingFinished(self):
        current_motor()[self.name] = self.number_widget.text()
        return super().editingFinished()
    
    def update(self):
        if not self.number_widget.hasFocus():
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
        self.checkbox.setChecked(float(val) != 0)
    
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
        val = float(current_motor()[self.name].get())
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
        self.slider.setValue(int(float(self.number_widget.text())))

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
        if self.button.text() in faults.keys():
            if faults[self.button.text()]:
                self.button.setStyleSheet("background-color: red")
            else:
                self.button.setStyleSheet("background-color: green")
        if self.button.text() in mask.keys():
            if not mask[self.button.text()]:
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
        self.field_names = sorted(current_motor().get_api_options())

        self.statuses = []
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        #layout.setContentsMargins(QMargins(0,0,0,0))
        for i in range(10):
            self.statuses.append(StatusCombo())
            self.statuses[i].layout.setContentsMargins(QMargins(0,0,0,0))
            layout.addWidget(self.statuses[i])
            self.statuses[i].signal.connect(self.valueEdit)
            self.statuses[i].combo_box.addItems(self.field_names)
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
        self.combo_box.addItems(["motor_position", "joint_position", "iq", "torque", "motor_velocity", "joint_velocity", "iq_desired", "reserved"])
        self.combo_box.currentIndexChanged.connect(lambda: self.series.removePoints(0,self.series.count()))
        self.layout.addWidget(self.combo_box)
        self.layout.addWidget(self.chart_view)
        var_layout = QHBoxLayout()
        self.pp = NumberDisplay("peak-peak")
        var_layout.addWidget(self.pp)
        self.std = NumberDisplay("std dev")
        var_layout.addWidget(self.std)
        self.mean = NumberDisplay("mean")
        var_layout.addWidget(self.mean)
        self.layout.addLayout(var_layout)
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
        self.t_seconds += (motor.diff_mcu_time(s.mcu_timestamp, self.mcu_timestamp))/cpu_frequency
        self.mcu_timestamp = s.mcu_timestamp
        self.series.append(self.t_seconds, getattr(s, self.combo_box.currentText()))
        val = np.array([d.y() for d in self.series.pointsVector()])
        self.std.setNumber(np.std(val))
        self.mean.setNumber(np.mean(val))
        self.pp.setNumber(max(val) - min(val))
        if len(self.series) > 500:
            self.series.remove(0)
        self.axis_x.setMax(self.t_seconds)
        self.axis_x.setMin(self.series.at(0).x())
        self.axis_y.setMin(min([d.y() for d in self.series.pointsVector()]))
        self.axis_y.setMax(max([d.y() for d in self.series.pointsVector()]))


class PlotTab2(MotorTab):
    global motor_manager

    def __init__(self):
        super(PlotTab2, self).__init__()

        self.name = "plot2"
        self.update_time = 10

        self.chart = QChart()
        self.chart_view = QChartView(self.chart)
        self.chart_view.setRubberBand(QChartView.VerticalRubberBand)
        self.layout = QVBoxLayout()
        self.combo_box = QComboBox()
        self.field_names = sorted(current_motor().get_api_options())
        self.combo_box.addItems(self.field_names)
        self.layout.addWidget(self.combo_box)
        self.combo_box.setCurrentText("vbus")
        self.layout.addWidget(self.chart_view)
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
        self.axis_y.setTitleText("Value")
        self.chart.addAxis(self.axis_y, Qt.AlignmentFlag.AlignLeft)
        self.series.attachAxis(self.axis_y)
        self.axis_y.setRange(-100,100)

        self.combo_box.currentIndexChanged.connect(lambda: self.series.removePoints(0,self.series.count()))

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
            self.series.append(self.t_seconds, float(val))
            val2 = np.array([d.y() for d in self.series.pointsVector()])
            self.std.setNumber(np.std(val2))
            self.mean.setNumber(np.mean(val2))
            self.pp.setNumber(max(val2) - min(val2))
            if len(self.series) > 500:
                self.series.remove(0)
            self.axis_y.setMin(min([d.y() for d in self.series.pointsVector()]))
            self.axis_y.setMax(max([d.y() for d in self.series.pointsVector()]))
            self.axis_x.setMax(self.t_seconds)
            self.axis_x.setMin(self.series.at(0).x())
        except ValueError:
            pass

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
        self.velocity_measured.setNumber(dp/dt)

        if (abs(dp/dt) < 10000):
            # reject rollovers
            try:
                self.series.append(self.t_seconds, dp/dt)
                if len(self.series) > 200:
                    self.series.remove(0)
                self.axis_y.setMin(min([d.y() for d in self.series.pointsVector()]))
                self.axis_y.setMax(max([d.y() for d in self.series.pointsVector()]))
                self.axis_x.setMax(self.t_seconds)
                self.axis_x.setMin(self.series.at(0).x())
            except ValueError:
                pass

    def velocity_update(self):
        p = float(self.widget.number_widget.text())
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

class CalibrateTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(CalibrateTab, self).__init__(*args, **kwargs)

        self.name = "calibrate"
        self.numbers = []

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.index_offset = APIDisplay("index_offset_measured")
        layout.addWidget(self.index_offset)

        self.current_d = APIDisplay("id", "id measured (A)")
        layout.addWidget(self.current_d)

        self.phase_lock_current_startup = APIDisplay("startup_phase_lock_current", "startup phase lock current (A)", tooltip="startup_param.phase_lock_current")
        layout.addWidget(self.phase_lock_current_startup)
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
        layout.addLayout(boollayout)
        
        

        self.obias = APIEdit("obias", "output bias (rad)", "main_loop_param.output_encoder.bias")
        self.obias.signal.connect(self.obias_set)
        self.oposition = NumberDisplay("output position (rad)", tooltip="status.joint_position")
        self.odir = APIDir("odir", "output position dir", "main_loop_param.output_encoder.dir")
        olayout = QHBoxLayout()
        olayout.addWidget(self.obias)
        olayout.addWidget(self.oposition)
        olayout.addWidget(self.odir)
        layout.addLayout(olayout)

        self.mbias = APIEdit("startup_mbias", "startup motor bias (rad)", "also calls set_startup_bias\nstartup_param.motor_encoder_bias")
        self.mbias.signal.connect(self.mbias_set)
        self.mposition = NumberDisplay("motor position (rad)", tooltip="status.motor_position")
        self.mdir = APIDir("mdir", "motor position dir", "fast_loop_param.motor_encoder.dir")

        mlayout = QHBoxLayout()
        mlayout.addWidget(self.mbias)
        mlayout.addWidget(self.mposition)
        mlayout.addWidget(self.mdir)
        layout.addLayout(mlayout)

        self.tbias = APIEdit("tbias", "torque bias (Nm)", "main_loop_param.torque_sensor.bias")
        self.torque = NumberDisplay("torque (Nm)", tooltip="status.torque")
        self.tdir = APIDir("tdir", "torque dir", "main_loop_param.torque_sensor.dir")
        tlayout = QHBoxLayout()
        tlayout.addWidget(self.tbias)
        tlayout.addWidget(self.torque)
        tlayout.addWidget(self.tdir)
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

        self.setLayout(layout)

    def update(self):
        super(CalibrateTab, self).update()
        self.index_offset.update()
        self.current_d.update()
        self.phase_lock_current_startup.update()
        self.oposition.setNumber(self.status.joint_position)
        self.mposition.setNumber(self.status.motor_position)
        self.odir.update()
        self.idir.update()
        self.torque.setNumber(self.status.torque)
        self.obias.update()
        self.mbias.update()
        self.mdir.update()
        self.phase_mode.update()
        self.position_limits_disable.update()
        self.tbias.update()
        self.tdir.update()

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

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        global motor_manager
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

        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.West)

        self.tuning_tab = QTabWidget()
        self.tuning_tab.setTabPosition(QTabWidget.TabPosition.West)
        self.tuning_tab.unpause = None


        self.tabs.addTab(FaultTab(), "fault")
        self.tabs.addTab(StatusTab(), "status")
        self.tabs.addTab(PlotTab(), "plot")
        self.tabs.addTab(PlotTab2(), "plot2")
        self.tabs.addTab(VelocityTab(), "velocity")
        self.tabs.addTab(LogTab(), "log")
        #self.tabs.addTab(self.tuning_tab, "tuning")
        self.tabs.addTab(CalibrateTab(), "calibrate")
        self.tabs.addTab(CurrentTuningTab(), "current tuning")
        self.tabs.addTab(PositionTuningTab(), "position tuning")

        self.setCentralWidget(self.tabs)
        self.last_tab = self.tabs.currentWidget()
        self.last_tab.unpause()

        self.last_tuning_tab = self.tuning_tab.currentWidget()

        self.tabs.currentChanged.connect(self.new_tab)
        self.tuning_tab.currentChanged.connect(self.new_tuning_tab)
        if "-fullscreen" in QCoreApplication.arguments():
            self.showFullScreen()

    def connect_motor(self, name):
         global cpu_frequency
         print("Connecting motor " + name)
         motor_manager.get_motors_by_name([name], allow_simulated = self.simulated)
         motor_manager.set_auto_count()
         self.setWindowTitle(name + " sn:" + current_motor().serial_number())
         cpu_frequency = current_motor().get_cpu_frequency()

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

class CurrentTuningTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(CurrentTuningTab, self).__init__(*args, **kwargs)

        self.update_time = 50
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
            t_seconds = data["timestamp"]/cpu_frequency*1000
            t_seconds -= t_seconds[0]

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
            valpha_beta = (Kc*np.block([va,vb,vc]).transpose()).transpose()
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

            self.phase.setNumber(-(phas_meas - phas_des)*180/np.pi)
            self.mag.setNumber(mag_meas/mag_des)

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
        self.command.current_tuning.amplitude = float(self.amplitude.number_widget.text())
        self.command_update()

    def bias_update(self):
        self.command.current_tuning.bias = float(self.bias.number_widget.text())
        self.command_update()

    def frequency_update(self):
        self.command.current_tuning.frequency = float(self.frequency.number_widget.text())
        self.command_update()

    def mode_update(self, selection):
        self.command.current_tuning.mode = int(motor.TuningMode.__members__[selection])
        self.command_update()

class StreamingChart(QChartView):
    def __init__(self, num_lines=1, *args, **kwargs):
        self.chart = QChart()
        super(StreamingChart, self).__init__(self.chart, *args, **kwargs)
        self.setRubberBand(QChartView.VerticalRubberBand)
        self.num_lines = num_lines  
        
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
            self.series[i] = QLineSeries()
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
                if len(self.series[i]) > 200:
                    self.series[i].remove(0)
                min1 = min(min1,min([d.y() for d in self.series[i].pointsVector()]))
                max1 = max(max1,max([d.y() for d in self.series[i].pointsVector()]))

            self.axis_y.setMin(min1)
            self.axis_y.setMax(max1)
            self.axis_x.setMax(t)
            self.axis_x.setMin(self.series[0].at(0).x())
        except ValueError:
            pass

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

        error = float(current_motor()["error"].get())
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
        self.command.position_tuning.amplitude = float(self.amplitude.number_widget.text())
        print("amplitude: {}".format(self.command.current_tuning.amplitude))
        self.command_update()

    def bias_update(self):
        self.command.position_tuning.bias = float(self.bias.number_widget.text())
        self.command_update()

    def frequency_update(self):
        self.command.position_tuning.frequency = float(self.frequency.number_widget.text())
        self.command_update()

    def mode_update(self, selection):
        self.command.position_tuning.mode = int(motor.TuningMode.__members__[selection])
        self.command_update()

app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()
