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
    QPlainTextEdit,
)
from PyQt5.QtGui import QPalette, QColor, QDoubleValidator
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
import motor

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

motor_manager = None


class NumberEdit(QWidget):
    signal = pyqtSignal(float)
    def __init__(self, name, description=None, value=0):
        super(NumberEdit, self).__init__()
        
        if description is None:
            description = name
        self.layout = QHBoxLayout()
        self.name = name
        self.layout.addWidget(QLabel(description))
        self.number_widget = QLineEdit()
        self.number_widget.setValidator(QDoubleValidator())
        self.setNumber(value)
        self.number_widget.editingFinished.connect(self.editingFinished)
        self.layout.addWidget(self.number_widget)
        self.setLayout(self.layout)
    
    def setNumber(self, number):
        self.number_widget.setText(str(number))

    def editingFinished(self):
        self.signal.emit(float(self.number_widget.text()))

class NumberDisplay(NumberEdit):
    def __init__(self, *args, **kwargs):
        super(NumberDisplay, self).__init__(*args, **kwargs)
        self.number_widget.setReadOnly(True)

class APIDisplay(NumberDisplay):
    def update(self):
        val = motor_manager.motors()[0][self.name].get()
        self.number_widget.setText(val)

class APIEdit(NumberEdit):
    def update(self):
        pass


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
        self.setNumber(motor_manager.motors()[0][self.name].get())

    def set_value(self, value):
        print("set {}={}".format(self.name, value))
        motor_manager.motors()[0][self.name] = str(value)

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
        outer_layout = QHBoxLayout()
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
        #self.field_names = motor_manager.motors()[0]["help"].get().split('\n')
        self.field_names = motor_manager.motors()[0].get_api_options()

        self.statuses = []
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)    
        layout.setContentsMargins(QMargins(0,0,0,0))
        for i in range(5):
            self.statuses.append(StatusCombo())
            layout.addWidget(self.statuses[i])
            self.statuses[i].signal.connect(self.valueEdit)
            self.statuses[i].combo_box.addItems(self.field_names)
        self.statuses[0].combo_box.setCurrentText("vbus")
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
        self.field_names = motor_manager.motors()[0]["help"].get().split('\n')
        self.combo_box.addItems(self.field_names)
        self.layout.addWidget(self.combo_box)
        self.layout.addWidget(self.chart_view)
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

        s = motor_manager.read()[0]
        self.mcu_timestamp = s.mcu_timestamp
        self.t_seconds = 0.0
      #  val = motor_manager.motors()[0][self.combo_box.currentText()].get()
      #  self.series.append(0, getattr(s, self.combo_box.currentText()))


    def update(self):
        super(PlotTab2, self).update()
        s = self.status
        self.t_seconds += (motor.diff_mcu_time(s.mcu_timestamp, self.mcu_timestamp))/170e6
        self.mcu_timestamp = s.mcu_timestamp
        val = motor_manager.motors()[0][self.combo_box.currentText()].get()
        try:
            self.series.append(self.t_seconds, float(val))
            if len(self.series) > 500:
                self.series.remove(0)
            self.axis_y.setMin(min([d.y() for d in self.series.points()]))
            self.axis_y.setMax(max([d.y() for d in self.series.points()]))
            self.axis_x.setMax(self.t_seconds)
            self.axis_x.setMin(self.series.at(0).x())
        except ValueError:
            pass

class VelocityTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(VelocityTab, self).__init__(*args, **kwargs)

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

        parameter_layout = QGridLayout()
        self.kp = ParameterEdit("vkp","kp")
        self.ki = ParameterEdit("vki","ki")
        self.max = ParameterEdit("vmax","current limit")
        self.max.signal.connect(lambda val: motor_manager.motors()[0]["vki_limit"].set(str(val)))
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
        layout.addWidget(self.velocity_measured)
        layout.addLayout(parameter_layout)
        self.setLayout(layout)
        self.mcu_timestamp = 0
        self.motor_position = 0

    def update(self):
        super(VelocityTab, self).update()
        dt = (motor.diff_mcu_time(self.status.mcu_timestamp, self.mcu_timestamp))/170e6
        self.mcu_timestamp = self.status.mcu_timestamp
        dp = self.status.motor_position - self.motor_position
        self.motor_position = self.status.motor_position
        self.velocity_measured.setNumber(dp/dt)

    def velocity_update(self):
        p = float(self.widget.number_widget.text())
        print("velocity command " + str(p))
        motor_manager.set_command_velocity([p])
        motor_manager.set_command_mode(motor.ModeDesired.Velocity)
        motor_manager.write_saved_commands()

    def set_imax(self, val):
        motor_manager.motors()[0]["imax"].set(str(val))
        motor_manager.motors()[0]["idmax"].set(str(val))
        motor_manager.motors()[0]["iki_limit"].set(str(val))
        motor_manager.motors()[0]["idki_limit"].set(str(val))

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
            log = motor_manager.motors()[0]["log"].get()
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

        self.current_d = APIDisplay("id")
        layout.addWidget(self.current_d)

        self.button = QPushButton("phase_lock")
        self.button.clicked.connect(self.phase_lock)
        layout.addWidget(self.button)

        self.setLayout(layout)

    def update(self):
        super(CalibrateTab, self).update()
        self.index_offset.update()
        self.current_d.update()
    
    def phase_lock(self):
        print("phase lock")
        motor_manager.set_command_mode(motor.ModeDesired.PhaseLock)
        motor_manager.set_command_current([10])
        motor_manager.write_saved_commands()


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
        actions = []
        for m in motors:
            actions.append(self.motor_menu.addAction(m.name()))
            print(m.name())
            actions[-1].triggered.connect(lambda: self.connect_motor(m.name()))
        self.connect_motor(motors[0].name())
        print(actions)
        


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
         print("Connecting motor " + name)
         motor_manager.get_motors_by_name([name])
         motor_manager.set_auto_count()
         self.setWindowTitle(name + " sn:" + motor_manager.motors()[0].serial_number())

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

        self.name = "current_tuning"
        self.command = motor.Command()
        self.command.mode_desired = motor.ModeDesired.CurrentTuning
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.amplitude = NumberEditSlider("amplitude")
        self.amplitude.slider.setMinimum(-100)
        self.amplitude.slider.setMaximum(100)
        self.amplitude.slider.setPageStep(5)       
        self.amplitude.signal.connect(self.amplitude_update)
        layout.addWidget(self.amplitude)

        self.bias = NumberEditSlider("bias")
        self.bias.slider.setMinimum(-100)
        self.bias.slider.setMaximum(100)
        self.bias.slider.setPageStep(5)       
        self.bias.signal.connect(self.bias_update)
        layout.addWidget(self.bias)

        self.frequency = NumberEditSlider("frequency")
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
        self.kp = ParameterEdit("ikp")
        self.kp.signal.connect(lambda val: motor_manager.motors()[0]["idkp"].set(str(val)))
        self.ki = ParameterEdit("iki")
        self.kp.signal.connect(lambda val: motor_manager.motors()[0]["idki"].set(str(val)))
        self.ki_limit = ParameterEdit("iki_limit")
        self.kp.signal.connect(lambda val: motor_manager.motors()[0]["idki_limit"].set(str(val)))
        self.command_max = ParameterEdit("imax")
        self.kp.signal.connect(lambda val: motor_manager.motors()[0]["idmax"].set(str(val)))
        parameter_layout.addWidget(self.kp,0,0)
        parameter_layout.addWidget(self.ki,0,1)
        parameter_layout.addWidget(self.ki_limit,1,0)
        parameter_layout.addWidget(self.command_max,1,1)
        layout.addLayout(parameter_layout)

        self.text = QPlainTextEdit()
        layout.addWidget(self.text)
        self.setLayout(layout)

    def update(self):
        super(CurrentTuningTab, self).update()
        self.text.setPlainText(motor_manager.motors()[0].get_fast_log())

    def unpause(self):
        self.kp.refresh_value()
        self.ki.refresh_value()
        self.ki_limit.refresh_value()
        self.command_max.refresh_value()
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


class PositionTuningTab(MotorTab):
    def __init__(self, *args, **kwargs):
        super(PositionTuningTab, self).__init__(*args, **kwargs)

        self.name = "position_tuning"
        self.command = motor.Command()
        self.command.mode_desired = motor.ModeDesired.PositionTuning
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.amplitude = NumberEditSlider("amplitude")
        self.amplitude.slider.setMinimum(-100)
        self.amplitude.slider.setMaximum(100)
        self.amplitude.slider.setPageStep(5)       
        self.amplitude.signal.connect(self.amplitude_update)
        layout.addWidget(self.amplitude)

        self.bias = NumberEditSlider("bias")
        self.bias.slider.setMinimum(-100)
        self.bias.slider.setMaximum(100)
        self.bias.slider.setPageStep(5)       
        self.bias.signal.connect(self.bias_update)
        layout.addWidget(self.bias)

        self.frequency = NumberEditSlider("frequency")
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
        self.kp = ParameterEdit("kp")
        self.kd = ParameterEdit("kd")
        #self.ki_limit = ParameterEdit("ki_limit")
        self.command_max = ParameterEdit("max")
        parameter_layout.addWidget(self.kp,0,0)
        parameter_layout.addWidget(self.kd,0,1)
        #parameter_layout.addWidget(self.ki_limit,1,0)
        parameter_layout.addWidget(self.command_max,1,0)
        layout.addLayout(parameter_layout)

        self.setLayout(layout)

    def update(self):
        super(PositionTuningTab, self).update()

    def unpause(self):
        self.kp.refresh_value()
        self.kd.refresh_value()
        self.command_max.refresh_value()
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

app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()
