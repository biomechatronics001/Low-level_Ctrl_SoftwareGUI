'''
Entry 05/06/2024
This code is based on the runnig version of "GUI_4_Ctrl_v2.py" and "HipExo_GUI_v11.py".

The following was added/modified:
1. This GUI allows to connect to myactuator's RMD-L-4010 22T
3. Display real-time data (angular position)
4. Allow send a position command

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
fps = 50
Plots_RefreshRate = int(1000/fps) # in miliseconds [this value must be in accordance with the transmission rate defined in the teensy code -- Must be >= than the transmission rate e.g. In Teensy the value is 30 samples per second. Therefore the minimum value for this value must be 30 ms = 33.33 fps. However, values minor to 20 ms have negative results in the responsiveness of the GUI.]
win_size  = 1000 # Quantity of data points to be displayed in the GUI when real-time plotting [e.g. 300 = 10 seconds of data been displayed. This value is linked to the transmission rate defined on Teensy.]
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
cmd = 0

red  = pg.mkPen(color=(255, 0, 0), width = 2)
blue = pg.mkPen(color=(0, 0, 255), width = 2)

Connection_Flag    = False
LogginButton_Flag  = False
TRefBox_Flag       = False
Transfer_Flag      = False
Data_Received_Flag = False
first_teensy_time  = True
t = 0
t_teensy = 0
t_minus_1 = 0
t_0   = 0
tau_d = 0
q_d   = 0

M_Selected = 0
CtrlMode_Selected = 0
CtrlMode_Selected_str = " "

class MainWindow(QWidget):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            ConnectButton, LoggingButton, SerialComboBox,\
            Connection_Flag, connected_ports,\
            M1_Rbutton, M2_Rbutton, M3_Rbutton, M4_Rbutton, M5_Rbutton,\
            CtrlPos_Rbutton, CtrlVel_Rbutton, CtrlTor_Rbutton, CtrlImp_Rbutton,\
            Cmd_text
                
        self.setWindowTitle('GUI for Control v2.0')
        
        connected_ports = find_available_ports()

        # Layout definition
        MainLayout      = QHBoxLayout() # Window Layout
        Ctrls_Layout    = QVBoxLayout() # Layout for the controls on the left side of the window
        Comm_Layout     = QHBoxLayout() # Layout for the COM pot selection and the Log data button
        Select_Layout   = QHBoxLayout() # Main Layout for the selection of motor and control mode
        Motors_Layout   = QVBoxLayout() # Motor selection Layout
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
        Motors       = QGroupBox("Motor")
        Control_Mode = QGroupBox("Control Mode")
        ## Objects
        ### Motors
        M1_Rbutton = QRadioButton("1")
        M2_Rbutton = QRadioButton("2")
        M3_Rbutton = QRadioButton("3")
        M4_Rbutton = QRadioButton("4")
        M5_Rbutton = QRadioButton("5")
        ### Control Modes
        CtrlPos_Rbutton = QRadioButton("Position Control")
        CtrlVel_Rbutton = QRadioButton("Velocity Control")
        CtrlTor_Rbutton = QRadioButton("Torque Control")
        CtrlImp_Rbutton = QRadioButton("Impedance Control")
        
        ## Arrangement
        ### Motors
        Motors_Layout.addWidget(M1_Rbutton)
        Motors_Layout.addWidget(M2_Rbutton)
        Motors_Layout.addWidget(M3_Rbutton)
        Motors_Layout.addWidget(M4_Rbutton)
        Motors_Layout.addWidget(M5_Rbutton)
        Motors.setLayout(Motors_Layout)
        Select_Layout.addWidget(Motors)
        ### Control Modes
        CtrlMode_Layout.addWidget(CtrlPos_Rbutton)
        CtrlMode_Layout.addWidget(CtrlVel_Rbutton)
        CtrlMode_Layout.addWidget(CtrlTor_Rbutton)
        CtrlMode_Layout.addWidget(CtrlImp_Rbutton)
        Control_Mode.setLayout(CtrlMode_Layout)
        Select_Layout.addWidget(Control_Mode)

        # Cmd_Layout objects (creation & arrangement)
        ## Objects
        Cmd_Layout.addWidget(QLabel("Command"))
        Cmd_text  = QLineEdit()
        CmdButton = QPushButton("Send")
        ## Arrangement
        Cmd_Layout.addWidget(Cmd_text)
        Cmd_Layout.addWidget(CmdButton)

        # Plot_Layout objects (creation & arrangement)
        ## Objects (Real-time data displays)
        #LnR_IMU_plot = pg.PlotWidget()
        Position     = pg.PlotWidget()
        Velocity     = pg.PlotWidget()
        Torque       = pg.PlotWidget()
        Current      = pg.PlotWidget()
        ## Arrangement
        #Plot_Layout.addWidget(LnR_IMU_plot)
        Plot_Layout.addWidget(Position)
        Plot_Layout.addWidget(Velocity)
        Plot_Layout.addWidget(Torque)
        Plot_Layout.addWidget(Current)

        # Actions for the buttons
        ConnectButton.clicked.connect(Connect_Clicked)
        LoggingButton.clicked.connect(LogginButton_Clicked)
        CmdButton.clicked.connect(CmdButton_Clicked)

        # Configuring the look of the plots
        label_style = {"font-size": "16px"}
        title_style = {"color": "black", "font-size": "20px"}

        Position.setTitle("Angular Position", **title_style)
        Position.setLabel('left', "[deg]", **label_style)
        Position.setLabel('bottom', "Time [s]", **label_style)
        Position.addLegend()
        Position.setBackground('w')
        Position.showGrid(x=True, y=True)
        #self.L_Motor_Taud_line = Position.plot(t_buffer, L_motor_torque_d_buffer, name = "Command", pen = blue)
        self.L_Motor_Tau_line  = Position.plot(t_buffer, L_motor_torque_buffer, name = "Actual", pen = red)
        
        Velocity.setTitle("Angular velocity", **title_style)
        Velocity.setLabel('left', "[deg/s]", **label_style)
        Velocity.setLabel('bottom', "Time [s]", **label_style)
        Velocity.setBackground('w')
        Velocity.showGrid(x=True, y=True)
        self.L_motor_angpos_line = Velocity.plot(t_buffer, L_motor_angpos_buffer, pen = red)
        self.L_Motor_Taud_line   = Velocity.plot(t_buffer, L_motor_torque_d_buffer, name = "Command", pen = blue)

        Torque.setTitle("Torque", **title_style)
        Torque.setLabel('left', "[Nm]", **label_style)
        Torque.setLabel('bottom', "Time [s]", **label_style)
        Torque.addLegend()
        Torque.setBackground('w')
        Torque.showGrid(x=True, y=True)
        #self.R_Motor_Taud_line = Torque.plot(t_buffer, R_motor_torque_d_buffer, name = "Command", pen = blue)
        self.R_Motor_Tau_line  = Torque.plot(t_buffer, R_motor_torque_buffer, name = "Actual", pen = red)
        
        Current.setTitle("Current", **title_style)
        Current.setLabel('left', "[A]", **label_style)
        Current.setLabel('bottom', "Time [s]", **label_style)
        Current.setBackground('w')
        Current.showGrid(x=True, y=True)
        self.R_motor_angpos_line = Current.plot(t_buffer, R_motor_angpos_buffer, pen = red)

        # Creation of the timer for executing the function repetitively
        self.timer = QtCore.QTimer()
        self.timer.setInterval(Plots_RefreshRate) # Set the refresh time-rate for the plotted data in the GUI (every x miliseconds)
        self.timer.timeout.connect(self.all) # This function is called 20 times per second [50Hz] (Fastests stable)
        self.timer.start()

    def all(self):
        global Connection_Flag, Data_Received_Flag
        
        if Connection_Flag:
            if ser.in_waiting > 0:
                ConnectButton.setText("Receiving")
                ConnectButton.setStyleSheet("background-color : green")
                Recieve_data()
                if Data_Received_Flag:
                    self.update_plot_data()
                    Data_Received_Flag = False
                
   
    def update_plot_data(self):
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer,\
            L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
            L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            Connection_Flag, LogginButton_Flag, TRefBox_Flag,\
            csv_file_name, DataHeaders, t_0_teensy, cmd

        if Connection_Flag == True:

            t = time.time() - t_0

            if t_minus_1 != t:
                
                t_buffer = t_buffer[1:]
                t_buffer.append(t_teensy - t_0_teensy)

                L_IMU_buffer = L_IMU_buffer[1:]
                L_IMU_buffer.append(L_leg_IMU_angle)

                R_IMU_buffer = R_IMU_buffer[1:]
                R_IMU_buffer.append(R_leg_IMU_angle)

                BattVolt_buffer = BattVolt_buffer[1:] # This is just for testing not true batt voltage
                BattVolt_buffer.append(t_teensy)

                L_motor_torque_d_buffer = L_motor_torque_d_buffer[1:]
                L_motor_torque_d_buffer.append(cmd)

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
            
                # self.L_IMU_line.setData(t_buffer, L_IMU_buffer)
                # self.R_IMU_line.setData(t_buffer, R_IMU_buffer)

                self.L_Motor_Taud_line.setData(t_buffer, L_motor_torque_d_buffer)
                self.L_Motor_Tau_line.setData(t_buffer, L_IMU_buffer)

                self.L_motor_angpos_line.setData(t_buffer, R_IMU_buffer)

                #self.R_Motor_Taud_line.setData(t_buffer, R_motor_torque_d_buffer)
                self.R_Motor_Tau_line.setData(t_buffer, L_motor_torque_buffer)

                self.R_motor_angpos_line.setData(t_buffer, R_motor_torque_buffer)

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
        global  ser, ble_datalength, data_length, decoded_data, Data_Received_Flag,\
                L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
                L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos,\
                t_0_teensy, first_teensy_time
        
        if ser.in_waiting >= 32:
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
                        
                        L_leg_IMU_angle        = decoded_data[0]
                        R_leg_IMU_angle        = decoded_data[1]
                        L_motor_torque         = decoded_data[2]
                        R_motor_torque         = decoded_data[3]
                        L_motor_torque_desired = decoded_data[4]
                        R_motor_torque_desired = decoded_data[5]
                        t_teensy               = decoded_data[6]
                        L_motor_angpos         = decoded_data[7]
                        R_motor_angpos         = decoded_data[8]

                        Data_Received_Flag = True
                        if first_teensy_time:
                            t_0_teensy = t_teensy
                            first_teensy_time = False
        else:
            print('Waiting for the whole data package...')
            # ConnectButton.setText("Searching Hip Exoskeleton")
            # ConnectButton.setStyleSheet("background-color : orange")



def Transmit_data():
    global rs232_datalength, data_package, ser, Transfer_Flag, cmd
        
    data_package = bytearray([165, 90, rs232_datalength, 0, 0, 0, 0, 0, int(cmd), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if ser.is_open:
        ser.write(data_package)
        #print(data_package)
    Transfer_Flag = False


def CmdButton_Clicked():
    global Transfer_Flag,\
        Cmd_text, cmd

    cmd = float(Cmd_text.text())    

    if cmd < -360 or cmd > 360:
        showdialog()
        cmd = saturation(cmd, -360, 360)
    
    Transmit_data()
    print("| Command " + str(cmd) + " sent |")


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


def Motor_CtrlMode_state():
    global M1_Rbutton, M2_Rbutton, M3_Rbutton, M4_Rbutton, M5_Rbutton,\
        CtrlPos_Rbutton, CtrlVel_Rbutton, CtrlTor_Rbutton, CtrlImp_Rbutton,\
        M_Selected, CtrlMode_Selected, CtrlMode_Selected_str

    if M1_Rbutton.isChecked() == True:
        M_Selected = 1
    elif M2_Rbutton.isChecked() == True:
        M_Selected = 2
    elif M3_Rbutton.isChecked() == True:
        M_Selected = 3
    elif M4_Rbutton.isChecked() == True:
        M_Selected = 4
    elif M5_Rbutton.isChecked() == True:
        M_Selected = 5

    if CtrlPos_Rbutton.isChecked() == True:
        CtrlMode_Selected = 1
        CtrlMode_Selected_str = "Position"
    elif CtrlVel_Rbutton.isChecked() == True:
        CtrlMode_Selected = 2
        CtrlMode_Selected_str = "Velocity"
    elif CtrlTor_Rbutton.isChecked() == True:
        CtrlMode_Selected = 3
        CtrlMode_Selected_str = "Torque"
    elif CtrlImp_Rbutton.isChecked() == True:
        CtrlMode_Selected = 4
        CtrlMode_Selected_str = "Impedance"

def showdialog():
   msg = QMessageBox()
   msg.setIcon(QMessageBox.Warning)    
   msg.setWindowTitle("Warning")
   msg.setText("Value out of bounds")
   msg.setInformativeText("Please introduce a numerical value between 0 and 2.")
   msg.setDetailedText("Every value minor to 0 will be taken as 0 and every value greater that 2 will be taken as 2")
   msg.setStandardButtons(QMessageBox.Ok)
   msg.exec_()

def saturation(value, min_value, max_value):
    return max(min_value, min(value, max_value))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    Window = MainWindow()
    Window.show()
    sys.exit(app.exec_())