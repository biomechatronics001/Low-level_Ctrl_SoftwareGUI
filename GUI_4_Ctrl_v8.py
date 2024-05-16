'''
Entry 05/14/2024
This code is based on the runnig version of "GUI_4_Ctrl_v6.py".

The following was added/modified:
1. Real-time display of position of two motors simultaneously

Expected beahvior and working

'''
import serial
from serial.tools import list_ports
import struct
import time
import datetime as dt
from math import *
import csv
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import json
import numpy as np

def find_available_ports():
    connected_ports = []

    # Get a list of all available ports
    available_ports = list(list_ports.comports())

    for port, desc, hwid in available_ports:
        try:
            # Attempt to open the serial port
            ser = serial.Serial(port)
            
            # Check if there is something connected to the port
            if ser.readable():
                connected_ports.append(port)
                
            # Close the serial port
            ser.close()
        except serial.SerialException:
            pass

    return connected_ports


fps = 30
Plots_RefreshRate = int(1000/fps) # in miliseconds [this value must be in accordance with the transmission rate defined in the teensy code -- Must be >= than the transmission rate e.g. In Teensy the value is 30 samples per second. Therefore the minimum value for this value must be 30 ms = 33.33 fps. However, values minor to 20 ms have negative results in the responsiveness of the GUI.]
Teensy_freq_ble   = 30
plot_time_window  = 10 # in seconds (the time lapse you want to see in the plots)
buffer_size  = plot_time_window * Teensy_freq_ble # Quantity of data points to be displayed in the GUI when real-time plotting [This value is linked to the transmission rate defined on Teensy.]

# Buffers creation
t_buffer          = list([0] * buffer_size)
M1_Pos_buffer     = t_buffer.copy()
M2_Pos_buffer     = t_buffer.copy()
M1_Pos_cmd_buffer = t_buffer.copy()
M2_Pos_cmd_buffer = t_buffer.copy()

# Variables related to the motor commands
M1_Pos = 0
M2_Pos = 0
M1_pos_cmd = 0
M2_pos_cmd = 0

# General variables used for sending values
pos_cmd     = 0
sw_bias     = 0
sw_amp      = 0
sw_freq     = 0
step_amp    = 0
step_t_high = 0
step_t_low  = 0

# Variables for time control
t_0    = 0 # Initial time
t_prev = 0 # Previuos time
t      = 0 # Current time
t_teensy = 0 # Time in the Teensy 4.1

# Boolean control variables (Event Flags)
Connection_Flag    = False
LogginButton_Flag  = False
Data_Received_Flag = False
first_teensy_time  = True

# Indexing selection variables
M_Selected           = 0
CtrlMode_Selected    = 0
Signal_Mode_Selected = 0
M1_gID = 0
M2_gID = 0

# Definition of the characteristics of the lines used in the plots
red  = pg.mkPen(color=(255, 0, 0), width = 2)
blue = pg.mkPen(color=(0, 0, 255), width = 2)


class MainWindow(QWidget):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        global t_buffer, M1_Pos_buffer, M2_Pos_buffer, M1_Pos_cmd_buffer, M2_Pos_cmd_buffer,\
            t_0, t_prev,\
            ConnectButton, LoggingButton, SerialComboBox,\
            Connection_Flag, connected_ports,\
            CtrlPos_Rbutton, CtrlVel_Rbutton, CtrlTor_Rbutton, CtrlImp_Rbutton,\
            Cmd_text, Control_Mode, Motors, Tabs,\
            M1_Constant_cmd_block, M1_Sinwave_cmd_block, M1_Step_cmd_block, M1_Slider,\
            M2_Constant_cmd_block, M2_Sinwave_cmd_block, M2_Step_cmd_block, M2_Slider,\
            sw_bias, sw_amp, sw_freq,\
            M1_sw_bias, M1_sw_amp, M1_sw_freq, M1_step_stepsize, M1_step_period, M1_step_dutycycle,\
            M2_sw_bias, M2_sw_amp, M2_sw_freq, M2_step_stepsize, M2_step_period, M2_step_dutycycle
                
        self.setWindowTitle('Motor Control Software V2.0')
        
        connected_ports = find_available_ports()

        # Layout definition
        MainLayout      = QHBoxLayout() # Window Layout
        Ctrls_Layout    = QVBoxLayout() # Layout for the controls on the left side of the window
        Comm_Layout     = QHBoxLayout() # Layout for the COM pot selection and the Log data button
        Select_Layout   = QVBoxLayout() # Main Layout for the selection of motor and control mode
        Motors_Layout   = QHBoxLayout() # Motor selection Layout
        CtrlMode_Layout = QVBoxLayout() # Control mode Layout
        Cmd_Layout      = QHBoxLayout() # Layout for sending commands
        Plot_Layout     = QVBoxLayout() # Layout for the Plots (Position, Velocity, and Torque)

        # Adding the sublayouts to the Main Layout
        MainLayout.addLayout(Ctrls_Layout)
        MainLayout.addLayout(Plot_Layout, stretch=5)

        # Adding the subsublayouts to the Sublayouts
        Ctrls_Layout.addLayout(Comm_Layout)
        Ctrls_Layout.addLayout(Select_Layout)
        Ctrls_Layout.addLayout(Cmd_Layout)

        # Set the Main window layout
        self.setLayout(MainLayout)

        # Ctrls_Layout objects (creation & arrangement)
        ## Objects
        ConnectButton  = QPushButton("Connect")
        SerialComboBox = QComboBox()
        LoggingButton  = QPushButton("Data Logging")
        ## Arrangement
        Comm_Layout.addWidget(QLabel("ComPort:"))
        Comm_Layout.addWidget(SerialComboBox)
        SerialComboBox.addItems(connected_ports)
        Comm_Layout.addWidget(ConnectButton)
        Comm_Layout.addWidget(LoggingButton)

        # Select_Layout objects (creation & arrangement)
        Control_Mode = QGroupBox("Control Mode")
        Motors       = QGroupBox("Motor")
        ### Control Modes
        CtrlPos_Rbutton = QRadioButton("Position Control")
        CtrlVel_Rbutton = QRadioButton("Velocity Control")
        CtrlTor_Rbutton = QRadioButton("Torque Control")
        CtrlImp_Rbutton = QRadioButton("Impedance Control")
        
        ## Arrangement
        ### Control Modes
        Control_Mode.setLayout(CtrlMode_Layout)
        Select_Layout.addWidget(Control_Mode)
        CtrlMode_Layout.addWidget(CtrlPos_Rbutton)
        CtrlMode_Layout.addWidget(CtrlVel_Rbutton)
        CtrlMode_Layout.addWidget(CtrlTor_Rbutton)
        CtrlMode_Layout.addWidget(CtrlImp_Rbutton)
        ### Motors
        # Creating the tabs for selecting the motors
        Tabs   = QTabWidget()
        Tab_M1 = QWidget() # Position control
        Tab_M2 = QWidget() # Velocity control
        Tab_M3 = QWidget() # Torque control
        Tab_M4 = QWidget() # Impedance control
        Tab_M5 = QWidget() # Impedance control
        # Adding the tabs to the Tabs container
        Tabs.addTab(Tab_M1, "1")
        Tabs.addTab(Tab_M2, "2")
        Tabs.addTab(Tab_M3, "3")
        Tabs.addTab(Tab_M4, "4")
        Tabs.addTab(Tab_M5, "5")
        Motors.setLayout(Motors_Layout)
        Select_Layout.addWidget(Motors)
        Motors_Layout.addWidget(Tabs)

        #########################################################################
        ############################ Motor 1 Tab ################################
        #########################################################################
        M1_Tab_Layout = QVBoxLayout()
        Tab_M1.setLayout(M1_Tab_Layout)
        ## Constant command block ##
        M1_Constant_cmd_block = QGroupBox("Constant command")
        M1_Constant_cmd_block_Layout = QGridLayout()
        M1_Constant_cmd_block.setLayout(M1_Constant_cmd_block_Layout)
        M1_Constant_cmd_block.setCheckable(True)
        M1_Constant_cmd_block.setChecked(False)
        M1_Tab_Layout.addWidget(M1_Constant_cmd_block)
        # Slider
        M1_Slider = QSlider(Qt.Horizontal)        
        M1_Constant_cmd_block_Layout.addWidget(M1_Slider,2,1,2,3)
        M1_Slider.setMinimum(0)
        M1_Slider.setMaximum(180)
        M1_Slider.setValue(10)
        M1_Slider.setTickPosition(QSlider.TicksBelow)
        M1_Slider.setTickInterval(1)
        M1_Slider.valueChanged.connect(M1_Slider_ValueChange)
        
        ## Sinwave command block ##
        M1_Sinwave_cmd_block = QGroupBox("Sinwave")
        M1_Sinwave_cmd_block_Layout = QGridLayout()
        M1_Sinwave_cmd_block.setLayout(M1_Sinwave_cmd_block_Layout)
        M1_Sinwave_cmd_block.setCheckable(True)
        M1_Sinwave_cmd_block.setChecked(False)
        M1_Tab_Layout.addWidget(M1_Sinwave_cmd_block)
        # Sinwave parameters selectors
        M1_sw_bias = QDoubleSpinBox()
        M1_sw_amp  = QDoubleSpinBox()
        M1_sw_freq = QDoubleSpinBox()
        M1_sw_bias.setValue(0)
        M1_sw_amp.setValue(0)
        M1_sw_freq.setValue(0)
        M1_sw_bias.valueChanged.connect(M1_Sinwave)
        M1_sw_amp.valueChanged.connect(M1_Sinwave)
        M1_sw_freq.valueChanged.connect(M1_Sinwave)
        M1_sw_bias_lb = QLabel("Bias (deg)")
        M1_sw_amp_lb  = QLabel("Amplitude (deg)")
        M1_sw_freq_lb = QLabel("Frequency (Hz)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_bias_lb, sw_label_row, 1)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_amp_lb, sw_label_row, 2)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_freq_lb, sw_label_row, 3)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_bias, sw_ctrls_row, 1)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_amp, sw_ctrls_row, 2)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_freq, sw_ctrls_row, 3)

        ## Step command block ##
        M1_Step_cmd_block        = QGroupBox("Step")
        M1_Step_cmd_block_Layout = QGridLayout()
        M1_Step_cmd_block.setLayout(M1_Step_cmd_block_Layout)
        M1_Step_cmd_block.setCheckable(True)
        M1_Step_cmd_block.setChecked(False)
        M1_Tab_Layout.addWidget(M1_Step_cmd_block)
        # Step parameters selectors
        M1_step_stepsize  = QDoubleSpinBox()
        M1_step_period    = QDoubleSpinBox()
        M1_step_dutycycle = QDoubleSpinBox()
        M1_step_stepsize.setValue(0)
        M1_step_period.setValue(0.01)
        M1_step_dutycycle.setValue(0)
        M1_step_stepsize.valueChanged.connect(M1_Step)
        M1_step_period.valueChanged.connect(M1_Step)
        M1_step_dutycycle.valueChanged.connect(M1_Step)
        M1_step_stepsize_lb  = QLabel("Step Size (deg)")
        M1_step_period_lb    = QLabel("Period (s)")
        M1_step_dutycycle_lb = QLabel("Duty Cycle (%)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M1_Step_cmd_block_Layout.addWidget(M1_step_stepsize_lb, sw_label_row, 1)
        M1_Step_cmd_block_Layout.addWidget(M1_step_period_lb, sw_label_row, 2)
        M1_Step_cmd_block_Layout.addWidget(M1_step_dutycycle_lb, sw_label_row, 3)
        M1_Step_cmd_block_Layout.addWidget(M1_step_stepsize, sw_ctrls_row, 1)
        M1_Step_cmd_block_Layout.addWidget(M1_step_period, sw_ctrls_row, 2)
        M1_Step_cmd_block_Layout.addWidget(M1_step_dutycycle, sw_ctrls_row, 3)

        #########################################################################
        ############################ Motor 2 Tab ################################
        #########################################################################
        M2_Tab_Layout = QVBoxLayout()
        Tab_M2.setLayout(M2_Tab_Layout)
        ## Constant command block ##
        M2_Constant_cmd_block = QGroupBox("Constant command")
        M2_Constant_cmd_block_Layout = QGridLayout()
        M2_Constant_cmd_block.setLayout(M2_Constant_cmd_block_Layout)
        M2_Constant_cmd_block.setCheckable(True)
        M2_Constant_cmd_block.setChecked(False)
        M2_Tab_Layout.addWidget(M2_Constant_cmd_block)
        # Slider
        M2_Slider = QSlider(Qt.Horizontal)        
        M2_Constant_cmd_block_Layout.addWidget(M2_Slider,2,1,2,3)
        M2_Slider.setMinimum(0)
        M2_Slider.setMaximum(180)
        M2_Slider.setValue(10)
        M2_Slider.setTickPosition(QSlider.TicksBelow)
        M2_Slider.setTickInterval(1)
        M2_Slider.valueChanged.connect(M2_Slider_ValueChange)
        
        ## Sinwave command block ##
        M2_Sinwave_cmd_block = QGroupBox("Sinwave")
        M2_Sinwave_cmd_block_Layout = QGridLayout()
        M2_Sinwave_cmd_block.setLayout(M2_Sinwave_cmd_block_Layout)
        M2_Sinwave_cmd_block.setCheckable(True)
        M2_Sinwave_cmd_block.setChecked(False)
        M2_Tab_Layout.addWidget(M2_Sinwave_cmd_block)
        # Sinwave parameters selectors
        M2_sw_bias = QDoubleSpinBox()
        M2_sw_amp  = QDoubleSpinBox()
        M2_sw_freq = QDoubleSpinBox()
        M2_sw_bias.setValue(0)
        M2_sw_amp.setValue(0)
        M2_sw_freq.setValue(0)
        M2_sw_bias.valueChanged.connect(M2_Sinwave)
        M2_sw_amp.valueChanged.connect(M2_Sinwave)
        M2_sw_freq.valueChanged.connect(M2_Sinwave)
        M2_sw_bias_lb = QLabel("Bias (deg)")
        M2_sw_amp_lb  = QLabel("Amplitude (deg)")
        M2_sw_freq_lb = QLabel("Frequency (Hz)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_bias_lb, sw_label_row, 1)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_amp_lb, sw_label_row, 2)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_freq_lb, sw_label_row, 3)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_bias, sw_ctrls_row, 1)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_amp, sw_ctrls_row, 2)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_freq, sw_ctrls_row, 3)

        ## Step command block ##
        M2_Step_cmd_block        = QGroupBox("Step")
        M2_Step_cmd_block_Layout = QGridLayout()
        M2_Step_cmd_block.setLayout(M2_Step_cmd_block_Layout)
        M2_Step_cmd_block.setCheckable(True)
        M2_Step_cmd_block.setChecked(False)
        M2_Tab_Layout.addWidget(M2_Step_cmd_block)
        # Step parameters selectors
        M2_step_stepsize  = QDoubleSpinBox()
        M2_step_period    = QDoubleSpinBox()
        M2_step_dutycycle = QDoubleSpinBox()
        M2_step_stepsize.setValue(0)
        M2_step_period.setValue(0.01)
        M2_step_dutycycle.setValue(0)
        M2_step_stepsize.valueChanged.connect(M2_Step)
        M2_step_period.valueChanged.connect(M2_Step)
        M2_step_dutycycle.valueChanged.connect(M2_Step)
        M2_step_stepsize_lb  = QLabel("Step Size (deg)")
        M2_step_period_lb    = QLabel("Period (s)")
        M2_step_dutycycle_lb = QLabel("Duty Cycle (%)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M2_Step_cmd_block_Layout.addWidget(M2_step_stepsize_lb, sw_label_row, 1)
        M2_Step_cmd_block_Layout.addWidget(M2_step_period_lb, sw_label_row, 2)
        M2_Step_cmd_block_Layout.addWidget(M2_step_dutycycle_lb, sw_label_row, 3)
        M2_Step_cmd_block_Layout.addWidget(M2_step_stepsize, sw_ctrls_row, 1)
        M2_Step_cmd_block_Layout.addWidget(M2_step_period, sw_ctrls_row, 2)
        M2_Step_cmd_block_Layout.addWidget(M2_step_dutycycle, sw_ctrls_row, 3)


        # Cmd_Layout objects (creation & arrangement)
        ## Objects
        # Cmd_Layout.addWidget(QLabel("Command"))
        # Cmd_text  = QLineEdit()
        # CmdButton = QPushButton("Send")
        # ## Arrangement
        # Cmd_Layout.addWidget(Cmd_text)
        # Cmd_Layout.addWidget(CmdButton)

        # Plot_Layout objects (creation & arrangement)
        ## Objects (Real-time data displays)
        M1_Pos_Plot = pg.PlotWidget()
        M2_Pos_Plot = pg.PlotWidget()
        ## Arrangement
        Plot_Layout.addWidget(M1_Pos_Plot)
        Plot_Layout.addWidget(M2_Pos_Plot)

        # Actions for the buttons
        ConnectButton.clicked.connect(Connect_Clicked)
        LoggingButton.clicked.connect(LogginButton_Clicked)
        # CmdButton.clicked.connect(CmdButton_Clicked)

        # Configuring the look of the plots
        label_style = {"font-size": "16px"}
        title_style = {"color": "black", "font-size": "20px"}

        M1_Pos_Plot.setTitle("M1 Angular Position", **title_style)
        M1_Pos_Plot.setLabel('left', "[deg]", **label_style)
        M1_Pos_Plot.setLabel('bottom', "Time [s]", **label_style)
        M1_Pos_Plot.addLegend()
        M1_Pos_Plot.setBackground('w')
        M1_Pos_Plot.showGrid(x=True, y=True)
        self.M1_pos_line     = M1_Pos_Plot.plot(t_buffer, M1_Pos_buffer, name = "Actual", pen = red)
        self.M1_pos_cmd_line = M1_Pos_Plot.plot(t_buffer, M1_Pos_cmd_buffer, name = "Command", pen = blue)
        
        M2_Pos_Plot.setTitle("M2 Angular Position", **title_style)
        M2_Pos_Plot.setLabel('left', "[deg]", **label_style)
        M2_Pos_Plot.setLabel('bottom', "Time [s]", **label_style)
        M2_Pos_Plot.addLegend()
        M2_Pos_Plot.setBackground('w')
        M2_Pos_Plot.showGrid(x=True, y=True)
        self.M2_pos_line     = M2_Pos_Plot.plot(t_buffer, M2_Pos_buffer, name = "Actual", pen = red)
        self.M2_pos_cmd_line = M2_Pos_Plot.plot(t_buffer, M2_Pos_cmd_buffer, name = "Command", pen = blue)

        # Creation of the timer for executing the function repetitively
        self.timer = QtCore.QTimer()
        self.timer.setInterval(Plots_RefreshRate) # Set the refresh time-rate for the plotted data in the GUI (every x miliseconds)
        self.timer.timeout.connect(self.all) # This function is called 20 times per second [50Hz] (Fastests stable)
        self.timer.start()

    def all(self):
        global Connection_Flag, Data_Received_Flag, M_Selected, CtrlMode_Selected

        General_State_Check()
        
        if Connection_Flag:
            if ser.in_waiting > 0:
                ConnectButton.setText("Receiving")
                ConnectButton.setStyleSheet("background-color : green")
                Recieve_data()
                if Data_Received_Flag:
                    CMD_Assignment()
                    self.update_plot_data()
                    Data_Received_Flag = False
                
   
    def update_plot_data(self):
        global t_buffer, M1_Pos_buffer, M2_Pos_buffer, M1_Pos_cmd_buffer, M2_Pos_cmd_buffer,\
            M1_Pos, M2_Pos, M1_pos_cmd, M2_pos_cmd,\
            t_0, t_prev, t_teensy, t_0_teensy,\
            Connection_Flag, LogginButton_Flag,\
            csv_file_name, DataHeaders, pos_cmd

        if Connection_Flag == True:

            t = time.time() - t_0

            if t_prev != t:

                # This section shift backwards the values on the buffer and add the new value at the last position of the buffer
                delta_t = t_teensy - t_0_teensy
                buffers = [t_buffer, M1_Pos_buffer, M1_Pos_cmd_buffer, M2_Pos_buffer, M2_Pos_cmd_buffer]
                values  = [delta_t, M1_Pos, M1_pos_cmd, M2_Pos, M2_pos_cmd]
                update_buffers(buffers, values)

                self.M1_pos_cmd_line.setData(t_buffer, M1_Pos_cmd_buffer)
                self.M1_pos_line.setData(t_buffer, M1_Pos_buffer)

                self.M2_pos_cmd_line.setData(t_buffer, M2_Pos_cmd_buffer)
                self.M2_pos_line.setData(t_buffer, M2_Pos_buffer)

                if LogginButton_Flag == True:
                    LoggedData = {
                        "time": t,
                        "L_IMU": L_leg_IMU_angle,
                        "R_IMU": R_leg_IMU_angle,
                        "L_Torque_d": L_motor_torque_desired,
                        "L_Torque": M1_Pos,
                        "L_AngPos": L_motor_angpos,
                        "R_Torque_d": R_motor_torque_desired,
                        "R_Torque": M2_Pos,
                        "R_AngPos": R_motor_angpos
                    }

                    with open(csv_file_name, mode="a", newline="") as file:
                        writer = csv.DictWriter(file, fieldnames = DataHeaders)
                        writer.writerow(LoggedData)
            t_prev = t
        else:
            print("NOT Connected")


def M1_Slider_ValueChange():
        global M1_Slider, M1_pos_cmd, pos_cmd

        # M1_pos_cmd = float(M1_Slider.value())

        if M1_pos_cmd < -360 or M1_pos_cmd > 360:
            showdialog()
            M1_pos_cmd = saturation(M1_pos_cmd, -360, 360)
        
        pos_cmd = float(M1_Slider.value())#M1_pos_cmd
        
        Transmit_data()


def M2_Slider_ValueChange():
        global M2_Slider, M2_pos_cmd, pos_cmd

        # M2_pos_cmd = float(M2_Slider.value())

        if M2_pos_cmd < -360 or M2_pos_cmd > 360:
            showdialog()
            M2_pos_cmd = saturation(M2_pos_cmd, -360, 360)
        
        pos_cmd = float(M2_Slider.value())#M2_pos_cmd
        
        Transmit_data()


def M1_Sinwave():
    global M1_sw_bias, M1_sw_amp, M1_sw_freq,\
            sw_bias, sw_amp, sw_freq
    
    sw_bias = int(M1_sw_bias.value())
    sw_amp  = int(M1_sw_amp.value())
    sw_freq = int(M1_sw_freq.value() * 100)

    Transmit_data()


def M2_Sinwave():
    global M2_sw_bias, M2_sw_amp, M2_sw_freq,\
            sw_bias, sw_amp, sw_freq
    
    sw_bias = int(M2_sw_bias.value())
    sw_amp  = int(M2_sw_amp.value())
    sw_freq = int(M2_sw_freq.value() * 100)

    Transmit_data()


def M1_Step():
    global M1_step_stepsize, M1_step_period, M1_step_dutycycle,\
            step_amp, step_t_high, step_t_low
    
    step_amp    = int(M1_step_stepsize.value())
    step_t_high = int(M1_step_period.value())
    step_t_low  = int(M1_step_dutycycle.value())

    Transmit_data()


def M2_Step():
    global M2_step_stepsize, M2_step_period, M2_step_dutycycle,\
            step_amp, step_t_high, step_t_low
    
    step_amp    = int(M2_step_stepsize.value())
    step_t_high = int(M2_step_period.value())
    step_t_low  = int(M2_step_dutycycle.value())

    Transmit_data()


def Connect_Clicked():
        global ConnectButton, ser, Connection_Flag, t_0, ble_datalength, data_length, decoded_data, rs232_datalength

        ### Defining the size of the received packages ###
        ble_datalength   = 32 # Recieved package data size
        rs232_datalength = 20 # Transmited package data size
        data_length      = ble_datalength - 3
        decoded_data     = [0]*data_length
        ##################################################

        #### Setting up the serial communication port and baudrate ###
        serial_port = SerialComboBox.currentText()
        baud_rate   = 115200
        ############################################

        ### Stablish the serial connection ###
        ser = serial.Serial(port=serial_port, baudrate=baud_rate)
        ser.timeout = 0 # set read timeout
        ######################################

        while not ser.is_open:
            print('Serial port closed')
        
        if ser.is_open:
            print('Serial port opened')
            ConnectButton.setText("Bluetooth activated")
            ConnectButton.setStyleSheet("background-color : yellow")
            Connection_Flag = True

            t_0 = time.time() # Set the initial time

def Recieve_data():
        global  ser, ble_datalength, data_length, decoded_data, Data_Received_Flag,\
                t_teensy, M1_Pos, M2_Pos, M3_Pos, M4_Pos, M5_Pos,\
                t_0_teensy, first_teensy_time
        
        if ser.in_waiting >= ble_datalength:
            if ser.read(1) == b'\xA5':  # 165 in uint8
                if ser.read(1) == b'\x5A':  # 90 in uint8
                    expected_length = ser.read(1)  # Read the length byte
                    if expected_length == bytes([ble_datalength]):  # Check the length
                        if ser.in_waiting >= data_length:  # Ensure enough data is available
                            coded_data = ser.read(data_length)
                            #decoded_data = [0] * (data_length // 2)  # Initialize decoded data array
                            decode_i = 0
                            for i in range(1, data_length, 2):
                                var = coded_data[i-1] + coded_data[i] * 256
                                var = (var - 65536) / 100.0 if var > 32767 else var / 100.0
                                decoded_data[decode_i] = var
                                decode_i += 1
                        
                            t_teensy = decoded_data[0]
                            M1_Pos   = decoded_data[1]
                            M2_Pos   = decoded_data[2]
                            M3_Pos   = decoded_data[3]
                            M4_Pos   = decoded_data[4]
                            M5_Pos   = decoded_data[5]

                            Data_Received_Flag = True
                            
                            if first_teensy_time:
                                t_0_teensy = t_teensy
                                first_teensy_time = False


def Transmit_data():
    global rs232_datalength, data_package, ser, pos_cmd, CtrlMode_Selected, M_Selected, Signal_Mode_Selected,\
        pos_cmd, sw_bias, sw_amp, sw_freq, step_amp, step_t_high, step_t_low
        
    data_package = bytearray([165, 90, rs232_datalength, CtrlMode_Selected, int(M_Selected), int(Signal_Mode_Selected), int(pos_cmd), int(sw_bias), sw_amp, sw_freq, step_amp, step_t_high, step_t_low, 0, 0, 0, 0, 0, 0, 0])
    if ser.is_open:
        ser.write(data_package)
        print("| Command " + str(pos_cmd) + " sent |")


def CmdButton_Clicked():
    global Cmd_text, pos_cmd

    pos_cmd = float(Cmd_text.text())    

    if pos_cmd < -360 or pos_cmd > 360:
        showdialog()
        pos_cmd = saturation(pos_cmd, -360, 360)
    
    Transmit_data()
    

def LogginButton_Clicked():
    global LogginButton_Flag, LoggingButton, csv_file_name, DataHeaders, t_0

    LogginButton_Flag = True

    t_0 = time.time()
    LoggingButton.setText("Logging data")
    LoggingButton.setStyleSheet("background-color : blue")
    csv_file_name = "GUI_Logger_" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    DataHeaders   = ["time", "L_IMU", "R_IMU", "L_Torque_d", "L_Torque", "L_AngPos", "R_Torque_d", "R_Torque", "R_AngPos"]

    # Create the CSV file and write the header
    with open(csv_file_name, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames = DataHeaders)
        writer.writeheader()


def General_State_Check():
    global CtrlPos_Rbutton, CtrlVel_Rbutton, CtrlTor_Rbutton, CtrlImp_Rbutton,\
        M_Selected, CtrlMode_Selected, Signal_Mode_Selected, \
        Control_Mode, Motors, Tabs,\
        M1_Constant_cmd_block, M1_Sinwave_cmd_block, M1_Step_cmd_block,\
        M1_gID, M2_gID
    
    if CtrlPos_Rbutton.isChecked() == True:
        CtrlMode_Selected = 1
    elif CtrlVel_Rbutton.isChecked() == True:
        CtrlMode_Selected = 2
    elif CtrlTor_Rbutton.isChecked() == True:
        CtrlMode_Selected = 3
    elif CtrlImp_Rbutton.isChecked() == True:
        CtrlMode_Selected = 4

    if (CtrlPos_Rbutton.isChecked() == True) or (CtrlVel_Rbutton.isChecked() == True) or (CtrlTor_Rbutton.isChecked() == True) or (CtrlImp_Rbutton.isChecked()):
        Motors.setEnabled(True)
        M_Selected = Tabs.currentIndex() + 1 # This check the tab active (the selected motor)
    else:
        Motors.setEnabled(False)

    if M1_Constant_cmd_block.isChecked() == True:
        Signal_Mode_Selected = 1        
    elif M1_Sinwave_cmd_block.isChecked() == True:
        Signal_Mode_Selected = 2
    elif M1_Step_cmd_block.isChecked() == True:
        Signal_Mode_Selected = 3

    if M2_Constant_cmd_block.isChecked() == True:
        Signal_Mode_Selected = 1
    elif M2_Sinwave_cmd_block.isChecked() == True:
        Signal_Mode_Selected = 2
    elif M2_Step_cmd_block.isChecked() == True:
        Signal_Mode_Selected = 3

    match M_Selected:
        case 1:
            match CtrlMode_Selected:
                case 1:
                    match Signal_Mode_Selected:
                        case 1:
                            M1_gID = 111
                        case 2:
                            M1_gID = 112
                        case 3:
                            M1_gID = 113

                case 2:
                    match Signal_Mode_Selected:
                        case 1:
                            M1_gID = 121
                        case 2:
                            M1_gID = 122
                        case 3:
                            M1_gID = 123

                case 3:
                    match Signal_Mode_Selected:
                        case 1:
                            M1_gID = 131
                        case 2:
                            M1_gID = 132
                        case 3:
                            M1_gID = 133

                case 4:
                    match Signal_Mode_Selected:
                        case 1:
                            M1_gID = 141
                        case 2:
                            M1_gID = 142
                        case 3:
                            M1_gID = 143

        case 2:
            match CtrlMode_Selected:
                case 1:
                    match Signal_Mode_Selected:
                        case 1:
                            M2_gID = 211
                        case 2:
                            M2_gID = 212
                        case 3:
                            M2_gID = 213

                case 2:
                    match Signal_Mode_Selected:
                        case 1:
                            M2_gID = 221
                        case 2:
                            M2_gID = 222
                        case 3:
                            M2_gID = 223

                case 3:
                    match Signal_Mode_Selected:
                        case 1:
                            M2_gID = 231
                        case 2:
                            M2_gID = 232
                        case 3:
                            M2_gID = 233

                case 4:
                    match Signal_Mode_Selected:
                        case 1:
                            M2_gID = 241
                        case 2:
                            M2_gID = 242
                        case 3:
                            M2_gID = 243
    
    print(str(M1_gID) + " | " + str(M2_gID))


def CMD_Assignment():
    global CtrlMode_Selected, M_Selected, Signal_Mode_Selected, pos_cmd, t_teensy,\
        M1_gID, M1_pos_cmd, M1_sw_bias, M1_sw_amp, M1_sw_freq,\
        M2_gID, M2_pos_cmd, M2_sw_bias, M2_sw_amp, M2_sw_freq

    match M1_gID:
        case 111:
            M1_pos_cmd = M1_Slider.value()
        case 112:
            M1_pos_cmd = M1_sw_bias.value() + M1_sw_amp.value() * sin((2 * pi * M1_sw_freq.value()) * (t_teensy))
        case 113:
            M1_pos_cmd = step_function(M1_step_stepsize.value(), M1_step_period.value(), M1_step_dutycycle.value(), t_teensy)

    match M2_gID:
        case 211:
            M2_pos_cmd = M2_Slider.value()
        case 212:
            M2_pos_cmd = M2_sw_bias.value() + M2_sw_amp.value() * sin((2 * pi * M2_sw_freq.value()) * (t_teensy))
        case 213:
            M2_pos_cmd = step_function(M2_step_stepsize.value(), M2_step_period.value(), M2_step_dutycycle.value(), t_teensy)


def showdialog():
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Warning)
    msg.setWindowTitle("Warning")
    msg.setText("Value out of bounds")
    msg.setInformativeText("Please introduce a numerical value between 0 and 2.")
    msg.setDetailedText("Every value minor to 0 will be taken as 0 and every value greater that 2 will be taken as 2")
    msg.setStandardButtons(QMessageBox.Ok)
    msg.exec_()


def update_buffers(buffers, values):
    for buffer, value in zip(buffers, values):
        buffer[:] = buffer[1:]  # This modifies the original buffer
        buffer.append(value)


def saturation(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def step_function(step_size, period, duty_cycle, t):

    t_high = period * (duty_cycle / 100)  # Calculate the high time based on the duty cycle
    mod_time = t % period         # Determine the position in the current cycle

    if mod_time < t_high:
        # If within the high duration, return the step size
        return step_size
    else:
        # Otherwise, return 0 for the low duration
        return 0


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    Window = MainWindow()
    Window.show()
    sys.exit(app.exec_())