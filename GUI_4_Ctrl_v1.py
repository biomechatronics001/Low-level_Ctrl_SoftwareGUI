'''
Entry 04/15/2024
This code is based on the runnig version of "HipExo_GUI_v8.py".

The following was added:

Expected beahvior and working

'''
import serial
from serial.tools import list_ports
import struct
import time
import datetime as dt
from math import *
import csv
from PyQt5 import QtWidgets, QtCore, QtGui
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import json
import numpy as np
import threading
#thread = threading.Thread(target=read_from_serial)
#thread.daemon = True
#thread.start()

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

win_size  = 150 # Quantity of data points to be displayed in the GUI when real-time plotting
t_buffer                = list([0] * win_size)
L_IMU_buffer            = t_buffer.copy()
R_IMU_buffer            = t_buffer.copy()
BattVolt_buffer         = t_buffer.copy()
L_motor_torque_buffer   = t_buffer.copy()
R_motor_torque_buffer   = t_buffer.copy()
L_motor_torque_d_buffer = t_buffer.copy()
R_motor_torque_d_buffer = t_buffer.copy()
L_motor_angpos_buffer   = t_buffer.copy()
R_motor_angpos_buffer   = t_buffer.copy()

L_leg_IMU_angle = 0
R_leg_IMU_angle = 0
L_motor_torque = 0
R_motor_torque = 0
L_motor_torque_desired = 0
R_motor_torque_desired = 0
L_motor_angpos = 0
R_motor_angpos = 0

red  = pg.mkPen(color=(255, 0, 0), width = 2)
blue = pg.mkPen(color=(0, 0, 255), width = 2)

Connection_Flag   = False
LogginButton_Flag = False
TRefBox_Flag      = False
Transfer_Flag     = False
t_teensy = 0
t_minus_1 = 0
t_0   = 0
tau_d = 0
q_d   = 0

class MainWindow(QWidget):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            ConnectButton, LoggingButton, SerialComboBox,\
            Connection_Flag, connected_ports
                
        self.setWindowTitle('GUI for Control v1.0')
        
        connected_ports = find_available_ports()

        # Layout definition
        MainLayout    = QHBoxLayout()
        TopLayout     = QHBoxLayout()
        BottomLayout  = QHBoxLayout()
        GenLeftLayout = QVBoxLayout()
        LMotorLayout  = QVBoxLayout()
        RMotorLayout  = QVBoxLayout()

        # Adding the sublayouts to the Main Layout
        MainLayout.addLayout(TopLayout)
        MainLayout.addLayout(BottomLayout, stretch=5)

        # Adding the subsublayouts to the Bottom Layouts
        BottomLayout.addLayout(GenLeftLayout)
        BottomLayout.addLayout(LMotorLayout, stretch=5)
        BottomLayout.addLayout(RMotorLayout, stretch=5)

        # Set the Main window layout
        self.setLayout(MainLayout)

        # Creating the interactive objects in the GUI (Top buttons)
        #   Buttons
        ConnectButton = QPushButton("Connect Bluetooth")
        LoggingButton = QPushButton("Start data Logging")

        # Creating the Plot objects (Real-time data displays)
        LnR_IMU_plot        = pg.PlotWidget()
        L_Motor_TnTd_plot   = pg.PlotWidget()
        L_Motor_AngPos_plot = pg.PlotWidget()
        R_Motor_TnTd_plot   = pg.PlotWidget()
        R_Motor_AngPos_plot = pg.PlotWidget()

        # Creating the tabs for selecting the control modes
        Tabs         = QTabWidget()
        Tab_Pos_ctrl = QWidget() # Position control
        Tab_Vel_ctrl = QWidget() # Velocity control
        Tab_Tau_ctrl = QWidget() # Torque control
        Tab_Imp_ctrl = QWidget() # Impedance control

        # Adding the tabs to the Tabs container
        Tabs.addTab(Tab_Pos_ctrl, "Position Control")
        Tabs.addTab(Tab_Vel_ctrl, "Velocity Control")
        Tabs.addTab(Tab_Tau_ctrl, "Torque Control")
        Tabs.addTab(Tab_Imp_ctrl, "Impedance Control")

        ##### Creating the structure of the tabs #####

        ### Position control tab layout ###
        Tab_Pos_ctrl_Layout = QVBoxLayout()
        Tab_Pos_ctrl.setLayout(Tab_Pos_ctrl_Layout)
        # Create and add the label for the section
        PosCtrl_Label = QLabel("Control Gains")
        Tab_Pos_ctrl_Layout.addWidget(PosCtrl_Label)
        # Create a widget to hold the form layout
        PosCtrl_Gains_Widget = QWidget()
        PosCtrl_Gains_Form   = QFormLayout(PosCtrl_Gains_Widget)
        Pos_kp_gain_Textbox = QLineEdit()
        Pos_kd_gain_Textbox = QLineEdit()
        Pos_ki_gain_Textbox = QLineEdit()
        PosCtrl_Gains_Form.addRow("kp", Pos_kp_gain_Textbox)
        PosCtrl_Gains_Form.addRow("kd", Pos_kd_gain_Textbox)
        PosCtrl_Gains_Form.addRow("ki", Pos_ki_gain_Textbox)
        # Add the widget, which contains the form, to the QVBoxLayout
        Tab_Pos_ctrl_Layout.addWidget(PosCtrl_Gains_Widget)
        # Create and add the send button
        PosCtrl_Send_Button = QPushButton("Set Gains")
        Tab_Pos_ctrl_Layout.addWidget(PosCtrl_Send_Button)

        ### Velocity control tab layout ###
        Tab_Vel_ctrl_Layout = QVBoxLayout()
        Tab_Vel_ctrl.setLayout(Tab_Vel_ctrl_Layout)
        # Create and add the label for the section
        VelCtrl_Label = QLabel("Control Gains")
        Tab_Vel_ctrl_Layout.addWidget(VelCtrl_Label)
        # Create a widget to hold the form layout
        VelCtrl_Gains_Widget = QWidget()
        VelCtrl_Gains_Form   = QFormLayout(VelCtrl_Gains_Widget)
        Vel_kp_gain_Textbox = QLineEdit()
        Vel_kd_gain_Textbox = QLineEdit()
        Vel_ki_gain_Textbox = QLineEdit()
        VelCtrl_Gains_Form.addRow("kp", Vel_kp_gain_Textbox)
        VelCtrl_Gains_Form.addRow("kd", Vel_kd_gain_Textbox)
        VelCtrl_Gains_Form.addRow("ki", Vel_ki_gain_Textbox)
        # Add the widget, which contains the form, to the QVBoxLayout
        Tab_Vel_ctrl_Layout.addWidget(VelCtrl_Gains_Widget)
        # Create and add the send button
        VelCtrl_Send_Button = QPushButton("Set Gains")
        Tab_Vel_ctrl_Layout.addWidget(VelCtrl_Send_Button)

        ### Torque control tab layout ###
        Tab_Tau_ctrl_Layout = QVBoxLayout()
        Tab_Tau_ctrl.setLayout(Tab_Tau_ctrl_Layout)
        # Create and add the label for the section
        TauCtrl_Label = QLabel("Control Gains")
        Tab_Tau_ctrl_Layout.addWidget(TauCtrl_Label)
        # Create a widget to hold the form layout
        TauCtrl_Gains_Widget = QWidget()
        TauCtrl_Gains_Form   = QFormLayout(TauCtrl_Gains_Widget)
        Tau_kp_gain_Textbox = QLineEdit()
        Tau_kd_gain_Textbox = QLineEdit()
        Tau_ki_gain_Textbox = QLineEdit()
        TauCtrl_Gains_Form.addRow("kp", Tau_kp_gain_Textbox)
        TauCtrl_Gains_Form.addRow("kd", Tau_kd_gain_Textbox)
        TauCtrl_Gains_Form.addRow("ki", Tau_ki_gain_Textbox)
        # Add the widget, which contains the form, to the QVBoxLayout
        Tab_Tau_ctrl_Layout.addWidget(TauCtrl_Gains_Widget)
        # Create and add the send button
        TauCtrl_Send_Button = QPushButton("Set Gains")
        Tab_Tau_ctrl_Layout.addWidget(TauCtrl_Send_Button)

        ### Impedance control tab layout ###
        Tab_Imp_ctrl_Layout = QVBoxLayout()
        Tab_Imp_ctrl.setLayout(Tab_Imp_ctrl_Layout)
        # Create and add the label for the section
        ImpCtrl_Label = QLabel("Control Gains")
        Tab_Imp_ctrl_Layout.addWidget(ImpCtrl_Label)
        # Create a widget to hold the form layout
        ImpCtrl_Gains_Widget = QWidget()
        ImpCtrl_Gains_Form   = QFormLayout(ImpCtrl_Gains_Widget)
        Imp_kp_gain_Textbox = QLineEdit()
        Imp_kd_gain_Textbox = QLineEdit()
        Imp_ki_gain_Textbox = QLineEdit()
        ImpCtrl_Gains_Form.addRow("kp", Imp_kp_gain_Textbox)
        ImpCtrl_Gains_Form.addRow("kd", Imp_kd_gain_Textbox)
        ImpCtrl_Gains_Form.addRow("ki", Imp_ki_gain_Textbox)
        # Add the widget, which contains the form, to the QVBoxLayout
        Tab_Imp_ctrl_Layout.addWidget(ImpCtrl_Gains_Widget)
        # Create and add the send button
        ImpCtrl_Send_Button = QPushButton("Set Gains")
        Tab_Imp_ctrl_Layout.addWidget(ImpCtrl_Send_Button)
        

        #   Text capture box
        #TRefBox = QLineEdit(self)

        #   Combo box
        SerialComboBox = QComboBox()

        # Adding functions to the interactive objects in the GUI
        ConnectButton.clicked.connect(Connect_Clicked)
        #TRefButton.clicked.connect(TorqueReference_Clicked)
        LoggingButton.clicked.connect(LogginButton_Clicked)


        # Setup of the widgets on each layout
        #   TOP LAYOUT Widgets
        TopLayout.addWidget(QLabel("ComPort:"))
        TopLayout.addWidget(SerialComboBox)
        SerialComboBox.addItems(connected_ports)
        TopLayout.addWidget(ConnectButton)
        TopLayout.addWidget(LoggingButton)

        #   BOTTOM GENERAL LAYOUT Widgets
        GenLeftLayout.addWidget(Tabs)
        GenLeftLayout.addWidget(LnR_IMU_plot)

        #   BOTTOM LEFT MOTOR LAYOUT Widgets
        LMotorLayout.addWidget(L_Motor_TnTd_plot)
        LMotorLayout.addWidget(L_Motor_AngPos_plot) 

        #   BOTTOM RIGHT MOTOR LAYOUT Widgets
        RMotorLayout.addWidget(R_Motor_TnTd_plot)
        RMotorLayout.addWidget(R_Motor_AngPos_plot)

               
        # Configuring the look of the plots
        label_style = {"font-size": "16px"}
        title_style = {"color": "black", "font-size": "20px"}

        LnR_IMU_plot.setTitle("IMU", **title_style)
        LnR_IMU_plot.setLabel('left', "Angle [deg]", **label_style)
        LnR_IMU_plot.setLabel('bottom', "Time [s]", **label_style)
        LnR_IMU_plot.addLegend()
        LnR_IMU_plot.setBackground('w')
        LnR_IMU_plot.showGrid(x=True, y=True)
        self.L_IMU_line = LnR_IMU_plot.plot(t_buffer, L_IMU_buffer, name = "Left IMU", pen = red)
        self.R_IMU_line = LnR_IMU_plot.plot(t_buffer, R_IMU_buffer, name = "Right IMU", pen = blue)

        L_Motor_TnTd_plot.setTitle("Motor Torque (Left)", **title_style)
        L_Motor_TnTd_plot.setLabel('left', "Torque [Nm]", **label_style)
        L_Motor_TnTd_plot.setLabel('bottom', "Time [s]", **label_style)
        L_Motor_TnTd_plot.addLegend()
        L_Motor_TnTd_plot.setBackground('w')
        L_Motor_TnTd_plot.showGrid(x=True, y=True)
        self.L_Motor_Taud_line = L_Motor_TnTd_plot.plot(t_buffer, L_motor_torque_d_buffer, name = "Command", pen = blue)
        self.L_Motor_Tau_line  = L_Motor_TnTd_plot.plot(t_buffer, L_motor_torque_buffer, name = "Actual", pen = red)
        
        L_Motor_AngPos_plot.setTitle("Motor Position (Left)", **title_style)
        L_Motor_AngPos_plot.setLabel('left', "Ang. Position [deg]", **label_style)
        L_Motor_AngPos_plot.setLabel('bottom', "Time [s]", **label_style)
        L_Motor_AngPos_plot.setBackground('w')
        L_Motor_AngPos_plot.showGrid(x=True, y=True)
        self.L_motor_angpos_line = L_Motor_AngPos_plot.plot(t_buffer, L_motor_angpos_buffer, pen = red)

        R_Motor_TnTd_plot.setTitle("Motor Torque (Right)", **title_style)
        R_Motor_TnTd_plot.setLabel('left', "Torque [Nm]", **label_style)
        R_Motor_TnTd_plot.setLabel('bottom', "Time [s]", **label_style)
        R_Motor_TnTd_plot.addLegend()
        R_Motor_TnTd_plot.setBackground('w')
        
        R_Motor_TnTd_plot.showGrid(x=True, y=True)
        self.R_Motor_Taud_line = R_Motor_TnTd_plot.plot(t_buffer, R_motor_torque_d_buffer, name = "Command", pen = blue)
        self.R_Motor_Tau_line  = R_Motor_TnTd_plot.plot(t_buffer, R_motor_torque_buffer, name = "Actual", pen = red)
        
        R_Motor_AngPos_plot.setTitle("Motor Position (Right)", **title_style)
        R_Motor_AngPos_plot.setLabel('left', "Ang. Position [deg]", **label_style)
        R_Motor_AngPos_plot.setLabel('bottom', "Time [s]", **label_style)
        R_Motor_AngPos_plot.setBackground('w')
        R_Motor_AngPos_plot.showGrid(x=True, y=True)
        self.R_motor_angpos_line = R_Motor_AngPos_plot.plot(t_buffer, R_motor_angpos_buffer, pen = red)

        # Creation of the timer for executing the function repetitively
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20) # Set the refresh time-rate for the plotted data in the GUI (every x miliseconds)
        self.timer.timeout.connect(self.all) # This function is called 20 times per second [50Hz] (Fastests stable)
        self.timer.start()

    def all(self):
        global Connection_Flag, Transfer_Flag
        
        if Connection_Flag:
            if ser.in_waiting > 0:
                ConnectButton.setText("Receiving")
                ConnectButton.setStyleSheet("background-color : green")
                Recieve_data()
                self.update_plot_data()
                if Transfer_Flag:
                    Send_gain_value()
                
   
    def update_plot_data(self):
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer,\
            L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
            L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            Connection_Flag, LogginButton_Flag, TRefBox_Flag,\
            csv_file_name, DataHeaders

        if Connection_Flag == True:

            t = time.time() - t_0

            if t_minus_1 != t:
                
                t_buffer = t_buffer[1:]
                t_buffer.append(t)

                L_IMU_buffer = L_IMU_buffer[1:]
                L_IMU_buffer.append(L_leg_IMU_angle)

                R_IMU_buffer = R_IMU_buffer[1:]
                R_IMU_buffer.append(R_leg_IMU_angle)

                BattVolt_buffer = BattVolt_buffer[1:] # This is just for testing not true batt voltage
                BattVolt_buffer.append(t_teensy)

                L_motor_torque_d_buffer = L_motor_torque_d_buffer[1:]
                L_motor_torque_d_buffer.append(L_motor_torque_desired)

                L_motor_torque_buffer = L_motor_torque_buffer[1:]
                L_motor_torque_buffer.append(L_motor_torque)

                L_motor_angpos_buffer = L_motor_angpos_buffer[1:]
                L_motor_angpos_buffer.append(L_motor_angpos)

                R_motor_torque_d_buffer = R_motor_torque_d_buffer[1:]
                R_motor_torque_d_buffer.append(R_motor_torque_desired)

                R_motor_torque_buffer = R_motor_torque_buffer[1:]
                R_motor_torque_buffer.append(R_motor_torque)

                R_motor_angpos_buffer = R_motor_angpos_buffer[1:]
                R_motor_angpos_buffer.append(R_motor_angpos)
            
                self.L_IMU_line.setData(t_buffer, L_IMU_buffer)
                self.R_IMU_line.setData(t_buffer, R_IMU_buffer)

                self.L_Motor_Taud_line.setData(t_buffer, L_motor_torque_d_buffer)
                self.L_Motor_Tau_line.setData(t_buffer, L_motor_torque_buffer)

                self.L_motor_angpos_line.setData(t_buffer, L_motor_angpos_buffer)

                self.R_Motor_Taud_line.setData(t_buffer, R_motor_torque_d_buffer)
                self.R_Motor_Tau_line.setData(t_buffer, R_motor_torque_buffer)

                self.R_motor_angpos_line.setData(t_buffer, R_motor_angpos_buffer)

                if LogginButton_Flag == True:
                    LoggedData = {
                        "time": t,
                        "L_IMU": L_leg_IMU_angle,
                        "R_IMU": R_leg_IMU_angle,
                        "L_Torque_d": L_motor_torque_desired,
                        "L_Torque": L_motor_torque,
                        "L_AngPos": L_motor_angpos,
                        "R_Torque_d": R_motor_torque_desired,
                        "R_Torque": R_motor_torque,
                        "R_AngPos": R_motor_angpos
                    }

                    with open(csv_file_name, mode="a", newline="") as file:
                        writer = csv.DictWriter(file, fieldnames = DataHeaders)
                        writer.writerow(LoggedData)
            t_minus_1 = t
        else:
            print("NOT Connected")


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
        global  ser, ble_datalength, data_length, decoded_data,\
                L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
                L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos
        if ser.in_waiting > 0:
            if ser.read(1) == b'\xA5':  # 165 in uint8
                second_data_byte = ser.read(1)
                if second_data_byte == b'\x5A':  # 90 in uint8
                    if ser.read(1) == bytes([ble_datalength]):
                        
                        coded_data = ser.read(data_length)
                        decode_i = 0
                        for i in range(1, data_length, 2):
                            var = coded_data[i-1] + coded_data[i]*256
                            var = (var - 65536)/100.0 if var > 32767 else var/100.0
                            decoded_data[decode_i] = var
                            decode_i += 1
                        
                        L_leg_IMU_angle        = decoded_data[0]
                        R_leg_IMU_angle        = decoded_data[1]
                        L_motor_torque         = decoded_data[2]
                        R_motor_torque         = decoded_data[3]
                        L_motor_torque_desired = decoded_data[4]
                        R_motor_torque_desired = decoded_data[5]
                        t_teensy               = decoded_data[6]
                        L_motor_angpos         = decoded_data[7]
                        R_motor_angpos         = decoded_data[8]

                        print(t_teensy, L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque)
        else:
            print('no data received')
            ConnectButton.setText("Searching Hip Exoskeleton")
            ConnectButton.setStyleSheet("background-color : orange")

def Send_gain_value():
    global TRefBox, hip_exo_gain, rs232_datalength, data_package, ser, Transfer_Flag
    if Transfer_Flag:
        ser.close()
        hip_exo_gain = int(TRefBox.text())
        data_package = bytearray([165, 90, rs232_datalength, hip_exo_gain, 0, hip_exo_gain, 0, hip_exo_gain, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        ser.open()
        if ser.is_open:
            ser.write(data_package)
            print(data_package)
        Transfer_Flag = False

def TorqueReference_Clicked():
    global Transfer_Flag
    #global TRefBox, TRefBox_Flag, TRefBox_Command
    #TRefBox_Flag = True
    #TRefBox_Command = str(TRefBox.text())
    Transfer_Flag = True
    #Send_gain_value()

def LogginButton_Clicked():
    global LogginButton_Flag, LoggingButton, csv_file_name, DataHeaders, t_0
    LogginButton_Flag = True
    t_0 = time.time()
    LoggingButton.setText("Logging data")
    LoggingButton.setStyleSheet("background-color : blue")
    csv_file_name = "GUI_Logger_" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    DataHeaders = ["time", "L_IMU", "R_IMU", "L_Torque_d", "L_Torque", "L_AngPos", "R_Torque_d", "R_Torque", "R_AngPos"]

    # Create the CSV file and write the header
    with open(csv_file_name, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames = DataHeaders)
        writer.writeheader()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    Window = MainWindow()
    Window.show()
    sys.exit(app.exec_())