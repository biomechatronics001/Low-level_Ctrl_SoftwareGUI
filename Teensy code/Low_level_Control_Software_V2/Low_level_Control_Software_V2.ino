// 2024/05/06
// Mofied by Ivan Lopez-Sanchez to achieve wireless communication with a GUI

#include "Gemini_Teensy41.h"
#include <FlexCAN_T4.h>
#include <SD.h>
#include <cmath> // Include cmath for fmod

const int chipSelect = BUILTIN_SDCARD;  // Teensy 4.1 uses the built-in SD card slot
File dataFile; // Declare dataFile as a global variable

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;

uint32_t ID_offset = 0x140;
uint32_t motor_ID1 = 1; // Motor Can Bus ID 1 for guidewire rotation
uint32_t motor_ID2 = 5;//2; // Motor Can Bus ID 2 for guidewire insertion
uint32_t motor_ID3 = 6; // Motor Can Bus ID 3 for microcatheter (motor #3)
uint32_t motor_ID4 = 4; // Motor Can Bus ID 4 for guidcatheter insertion;
uint32_t motor_ID5 = 8; // Motor Can Bus ID 5 for guidecatheter rotation
uint32_t motor_ID6 = 0; // Motor Can Bus ID 5 for guidecatheter rotation

int CAN_ID = 3;  // CAN port from Teensy
double Gear_ratio = 1;

Gemini_Teensy41 m1(motor_ID1, CAN_ID, Gear_ratio);
Gemini_Teensy41 m2(motor_ID2, CAN_ID, Gear_ratio);
Gemini_Teensy41 m3(motor_ID3, CAN_ID, Gear_ratio);
Gemini_Teensy41 m4(motor_ID4, CAN_ID, Gear_ratio);
Gemini_Teensy41 m5(motor_ID5, CAN_ID, Gear_ratio);
Gemini_Teensy41 m6(motor_ID6, CAN_ID, Gear_ratio);

//***For managing the Controller and Bluetooth rate
double freq_ctrl  = 100;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double freq_ble   = 30;  // [Hz] Bluetooth sending data frequency
unsigned long t_0 = 0;
unsigned long t   = 0;
unsigned long prev_t_ctrl = 0;                                      // used to control the controller sample rate.
unsigned long prev_t_ble  = 0;                                      // used to control the Bluetooth communication frequency
unsigned long T_ctrl_micros = (unsigned long)(1000000 / freq_ctrl); // used to control the teensy controller frequency
unsigned long T_ble_micros  = (unsigned long)(1000000 / freq_ble);  // used to control the Bluetooth communication frequency
//**********************************

//***Data sent via bluetooth
char datalength_ble    = 32;      // Bluetooth Data Length (32)
char data_ble[60]      = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy

int M1_pos = 0;
int M2_pos = 0;
int M3_pos = 0;
int M4_pos = 0;
int M5_pos = 0;

int t_teensy = 0;

int M_Selected           = 0;
int CtrlMode_Selected    = 0;
int Signal_Mode_Selected = 0;

float pos_cmd        = 0;
float sw_bias        = 0;
float sw_amp         = 0;
float sw_freq        = 0;
float step_size      = 0;
float step_period    = 0;
float step_dutycycle = 0;

int M1_g_ID = 0;
int M2_g_ID = 0;
int M_active = 0;

double m1_pos_cmd = 0.0;
double m2_pos_cmd = 0.0;
double m3_pos_cmd = 0.0;
double m4_pos_cmd = 0.0;
double m5_pos_cmd = 0.0;
double m6_pos_cmd = 0.0;

double m1_sw_bias        = 0;
double m1_sw_amp         = 0;
double m1_sw_freq        = 0;
double m1_step_size      = 0;
double m1_step_period    = 0;
double m1_step_dutycycle = 0;

double m2_sw_bias        = 0;
double m2_sw_amp         = 0;
double m2_sw_freq        = 0;
double m2_step_size      = 0;
double m2_step_period    = 0;
double m2_step_dutycycle = 0;
//**************************

// Linear Actuator
//const int forwardPin = 3;
//const int backwardPin = 4;
//
//double pos_Lin = 0;      // linear actuator position reading
//double pos_pr_Lin = 0;   // linear actuator prior position
//double pos_dot_Lin = 0;  // linear actuator velocit
//
//double setpoint = 0;     // target value
//double setpointnow = 0;  //
//
////PD control for the actuation
//const double kp = 12;
//const double kd = 0.4;
//double cmd = 0;    // command for linear actuator
//double error = 0;  // error signal for position
//
//
//const double con_Num = 0.12135;  // calculated value for the linear actuator (different voltage different value)


double cur_Command_1 = 0;
double vel_Command_1 = 0;
double pos_Command_1 = 0;

double cur_Command_2 = 0;
double vel_Command_2 = 0;
double pos_Command_2 = 0;

double cur_Command_3 = 0;
double vel_Command_3 = 0;
double pos_Command_3 = 0;

double cur_Command_4 = 0;
double vel_Command_4 = 0;
double pos_Command_4 = 0;

double cur_Command_5 = 0;
double vel_Command_5 = 0;
double pos_Command_5 = 0;

double cur_Command_6 = 0;
double vel_Command_6 = 0;
double pos_Command_6 = 0;

double pre_pos_motor1 = 0;
double pre_pos_motor2 = 0;
double pre_pos_motor3 = 0;
double pre_pos_motor4 = 0;
double pre_pos_motor5 = 0;
double pre_pos_motor6 = 0;

double Angle0 = 0;
double Angle1 = 0;
double Angle2 = 0;
double Angle3 = 0;
double Angle4 = 0;
double Angle5 = 0;

double delay_control = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.print("Setting up!");
  delay(100);
  Serial.begin(115200); //used for communication with computer.
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  // # may need to modify for the linear actuator?
  initial_CAN();
  delay(500);

  // will CAN3 available for motor or teensy?
    m1.init_motor(); // Strat the CAN bus communication & Motor Control
    m2.init_motor();
    m3.init_motor();
    m4.init_motor();
    m5.init_motor();
    m6.init_motor();

  delay(1000);
  reset_motor_angle();
  delay(1000);

    m1.read_PID();  //send read all PI Gain command for ID 1
    m2.read_PID();  //send read all PI Gain command for ID 2
    m3.read_PID();  //send read all PI Gain command for ID 4
    m4.read_PID();  //send read all PI Gain command for ID 7
    m5.read_PID();  //send read all PI Gain command for ID 8
    m6.read_PID();

  Serial.println("Set up complete.");

  //linear actuator
//  pinMode(forwardPin, OUTPUT);
//  pinMode(backwardPin, OUTPUT);
//  pinMode(A0, INPUT);
//  digitalWrite(forwardPin, LOW);
//  digitalWrite(backwardPin, HIGH);

  Serial.print("Controller executed at ");
  Serial.print(String(freq_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(freq_ble));
  Serial.println(" Hz");
  delay(5000);
  t_0 = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
//  setpoint = 200;//820
  t = micros() - t_0;

  if (t - prev_t_ctrl > T_ctrl_micros) {

    Angle0 = m1.motorAngle;
    Angle1 = m2.motorAngle;
    Angle2 = m3.motorAngle;
    Angle3 = m4.motorAngle;
    Angle4 = m5.motorAngle;
    Angle5 = m6.motorAngle;  

    if (t - prev_t_ble > T_ble_micros) {

      Receive_ble_Data();  // Receive ble data
      Transmit_ble_Data(); // Transmit ble data

      prev_t_ble = t;
    }
    //For tracking A*sin(2*pi*f*t); omega = 2*pi*f
    pos_Command_1 = 360 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.2 ); // f = 0.4 Hz input with 180 deg amplitude
    pos_Command_2 = 720 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.3);
    pos_Command_3 = 450 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.3); // for micro catheter
    pos_Command_4 = 360* 9 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.1);
    pos_Command_5 = 540 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.1);
    pos_Command_6 = 810 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.2);

    cdm_assignment();

    m1.send_position_command_2(m1_pos_cmd, 10000);  // send the command to mID1
    m2.send_position_command_2(m2_pos_cmd, 10000);  // send the command to mID2
    m3.send_position_command_2(m3_pos_cmd, 10000);  // send the command to mID4
    m4.send_position_command_2(m4_pos_cmd, 10000);  // send the command to mID7
    m5.send_position_command_2(m5_pos_cmd, 10000);  // send the command to mID8
    m6.send_position_command_2(m6_pos_cmd, 10000);

    // For mID1
    m1.receive_CAN_data();
    Wait(delay_control);

    m1.read_motor_status_3();
    m1.receive_CAN_data();
    Wait(delay_control);

    m1.read_multi_turns_angle();
    m1.receive_CAN_data();
    Wait(delay_control);

    // For mID2
    m2.receive_CAN_data();
    Wait(delay_control);
    
    m2.read_motor_status_3();
    m2.receive_CAN_data();
    Wait(delay_control);
    
    m2.read_multi_turns_angle();
    m2.receive_CAN_data();
    Wait(delay_control);
    
    // For mID4
    m3.receive_CAN_data();
    Wait(delay_control);
    
    m3.read_motor_status_3();
    m3.receive_CAN_data();
    Wait(delay_control);
    
    m3.read_multi_turns_angle();
    m3.receive_CAN_data();
    Wait(delay_control);

    // For mID7

    m4.receive_CAN_data();
    Wait(delay_control);
    
    m4.read_motor_status_3();
    m4.receive_CAN_data();
    Wait(delay_control);
    
    m4.read_multi_turns_angle();
    m4.receive_CAN_data();
    Wait(delay_control);

    //For mID8

    m5.receive_CAN_data();
    Wait(delay_control);
    
    m5.read_motor_status_3();
    m5.receive_CAN_data();
    Wait(delay_control);
    
    m5.read_multi_turns_angle();
    m5.receive_CAN_data();
    Wait(delay_control);

    //For mID8

    m6.receive_CAN_data();
    Wait(delay_control);
    
    m6.read_motor_status_3();
    m6.receive_CAN_data();
    Wait(delay_control);
    
    m6.read_multi_turns_angle();
    m6.receive_CAN_data();
    Wait(delay_control);

//    pre_pos_motor1 = m1.read_encoder;
//        pre_pos_motor2 = m2.read_encoder;
    //    pre_pos_motor3 = m3.read_encoder;
//        pre_pos_motor4 = m4.read_encoder;
//        pre_pos_motor5 = m5.read_encoder;

    // Plot the command and angle
    //set 0
    //set 0

    Angle0 = m1.motorAngle;
    Angle1 = m2.motorAngle;
    Angle2 = m3.motorAngle;
    Angle3 = m4.motorAngle;
    Angle4 = m5.motorAngle;
    Angle5 = m6.motorAngle;    
//    Serial.print(m1.motorAngle);
//    Serial.print(";");
    Serial.print(Angle1);
    Serial.print(";");
    Serial.print(Angle2);
    Serial.print(";");
    Serial.print(Angle3);
    Serial.print(";");
    Serial.print(Angle4);
    Serial.print(";");
    Serial.print(Angle5);
    Serial.println(";");


    // Serial.print(m1.motorAngle);
    // Serial.print(";");
    // Serial.print(m2.motorAngle);
    // Serial.print(";");
    // Serial.print(m3.motorAngle);
    // Serial.print(";");
    // Serial.print(m4.motorAngle);
    // Serial.print(";");
    // Serial.print(m5.motorAngle);
    // Serial.print(";");
    // Serial.print(m6.motorAngle);
    // Serial.println(";");
    
    //set 1
//    Serial.print(pos_Command_1);
//    Serial.print(",");
//    Serial.print(m1.motorAngle);
//    Serial.print(";");
//    Serial.print(pos_Command_2);
//    Serial.print(",");
//    Serial.print(m2.motorAngle);
//    Serial.print(";");
//    Serial.print(pos_Command_3);
//    Serial.print(",");
//    Serial.print(m3.motorAngle);
//    Serial.print(";");
//    Serial.print(pos_Command_4);
//    Serial.print(",");
//    Serial.print(m4.motorAngle);
//    Serial.print(";");
//    Serial.print(pos_Command_5);
//    Serial.print(",");
//    Serial.print(m5.motorAngle);
//    Serial.println(";");
//    
    //
    //set 2
    //    Serial.print(pos_Command_1);
    //    Serial.print(",");
    //    Serial.print(m1.motorAngle);
    ////    Serial.print(m1.read_multi_turns_angle);
    //    Serial.print(";");
    //    Serial.print(pos_Command_2);
    //    Serial.print(",");
    ////    Serial.print(m2.motorAngle_offset);
    //    Serial.print(m2.motorAngle);
    //    Serial.print(";");
    //    Serial.print(pos_Command_3);
    //    Serial.print(",");
    //    Serial.print(m3.motorAngle);
    //    Serial.print(";");
    ////    Serial.print(pos_Command_4);
    ////    Serial.print(",");
    ////    Serial.print(m4.motorAngle);
    ////    Serial.print(";");
    //    Serial.print(pos_Command_5);
    //    Serial.print(",");
    //    Serial.print(m5.motorAngle);
    //    Serial.println(";");
    
    //Linear Actuator Function Call
//    displayLinearData(pos_Lin, setpoint);
//    pos_pr_Lin = pos_Lin;
    
    prev_t_ctrl = t;
  }
}

//Linear Actuator Function 
//void displayLinearData(double pos_Lin, double setpoint) {
//  Serial.print(pos_Lin);  // for the linear actuator reading
//  Serial.print(" ; ");
//  Serial.print(setpoint);  // setting value
//  Serial.print(" ; ");
//}
////Linear Actuator Function 
//void forth(double v) {
//  analogWrite(forwardPin, v);
//  digitalWrite(backwardPin, LOW);
//}
////Linear Actuator Function 
//void back(double v) {
//  analogWrite(backwardPin, v);
//  digitalWrite(forwardPin, LOW);
//}

void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(500);
  Serial.println("Can bus setup done...");
  delay(500);
}

void reset_motor_angle() {
  for (int i = 0; i < 20; i++) {
    m1.read_multi_turns_angle();
    delay(10);
    m1.receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    delay(10);

    m2.read_multi_turns_angle();
    delay(10);
    m2.receive_CAN_data();
    m2.motorAngle_offset = m2.motorAngle_raw;
    delay(10);
//    
    m3.read_multi_turns_angle();
    delay(10);
    m3.receive_CAN_data();
    m3.motorAngle_offset = m3.motorAngle_raw;
    delay(10);
    
    m4.read_multi_turns_angle();
    delay(10);
    m4.receive_CAN_data();
    m4.motorAngle_offset = m4.motorAngle_raw;
    delay(10);

    m5.read_multi_turns_angle();
    delay(10);
    m5.receive_CAN_data();
    m5.motorAngle_offset = m5.motorAngle_raw;
    delay(10);

    m6.read_multi_turns_angle();
    delay(10);
    m6.receive_CAN_data();
    m6.motorAngle_offset = m6.motorAngle_raw;
    delay(10);
  }
}

void Wait(double delay_Control) {
  double time_Start = micros();
  double time_Delta = delay_Control;
  double time_Control = 0;

  do {
    time_Control = micros() - time_Start;
  } while (time_Control < time_Delta);
}

void tunning() {
  m1.write_PID_RAM();
  Wait(delay_control);
  m1.write_PID_RAM();
  Wait(delay_control);
  m1.write_PID_ROM();
  Wait(delay_control);
  m1.write_PID_ROM();
  Wait(delay_control);

  m2.write_PID_RAM();
  Wait(delay_control);
  m2.write_PID_RAM();
  Wait(delay_control);
  m2.write_PID_ROM();
  Wait(delay_control);
  m2.write_PID_ROM();
  Wait(delay_control);

  m3.write_PID_RAM();
  Wait(delay_control);
  m3.write_PID_RAM();
  Wait(delay_control);
  m3.write_PID_ROM();
  Wait(delay_control);
  m3.write_PID_ROM();
  Wait(delay_control);

  m4.write_PID_RAM();
  Wait(delay_control);
  m4.write_PID_RAM();
  Wait(delay_control);
  m4.write_PID_ROM();
  Wait(delay_control);
  m4.write_PID_ROM();
  Wait(delay_control);

  m5.write_PID_RAM();
  Wait(delay_control);
  m5.write_PID_RAM();
  Wait(delay_control);
  m5.write_PID_ROM();
  Wait(delay_control);
  m5.write_PID_ROM();
  Wait(delay_control);

  m6.write_PID_RAM();
  Wait(delay_control);
  m6.write_PID_RAM();
  Wait(delay_control);
  m6.write_PID_ROM();
  Wait(delay_control);
  m6.write_PID_ROM();
  Wait(delay_control);
}

void Receive_ble_Data(){
  if (Serial5.available() >= 20) {
    // Read the incoming byte:
    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165) { // Check the first byte

      if (data_rs232_rx[1] == 90) { // Check the second byte

        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          CtrlMode_Selected    = data_rs232_rx[3]; // 1:Position | 2:Velocity | 3:Torque | 4:Impedance
          M_Selected           = data_rs232_rx[4];
          Signal_Mode_Selected = data_rs232_rx[5]; // 1:constant | 2:sinwave | 3:step
          pos_cmd              = data_rs232_rx[6];
          sw_bias              = data_rs232_rx[7];
          sw_amp               = data_rs232_rx[8];
          sw_freq              = data_rs232_rx[9] / 100.0;
          step_size            = data_rs232_rx[10];
          step_period          = data_rs232_rx[11];
          step_dutycycle       = data_rs232_rx[12];

          M_active = M_Selected;

          g_ID_assignment();

          Serial.print("| Motor ");
          Serial.print(M_Selected, DEC); // This contains the motor number
          Serial.print(" selected | Control Mode ");
          Serial.print(CtrlMode_Selected, DEC); // This contains the Control mode
          Serial.print(" selected | Signal ");
          Serial.print(Signal_Mode_Selected, DEC); // This contains the Control mode
          Serial.print(" selected | Command ");
          Serial.print(pos_cmd, DEC);
          Serial.println(" sent |");
        }
      }
    }
  }
}

void Transmit_ble_Data(){
  t_teensy = (t / 10000.0); // this is the same as ((t / 1000000.0) * 100)
  M1_pos = Angle1 * 100;
  M2_pos = Angle2 * 100;
  M3_pos = 100;
  M4_pos = 100;
  M5_pos = 100;

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8;
  data_ble[5]  = M1_pos;
  data_ble[6]  = M1_pos >> 8;
  data_ble[7]  = M2_pos;
  data_ble[8]  = M2_pos >> 8;
  data_ble[9]  = M3_pos;
  data_ble[10] = M3_pos >> 8;
  data_ble[11] = M4_pos;
  data_ble[12] = M4_pos >> 8;
  data_ble[13] = M5_pos;
  data_ble[14] = M5_pos >> 8;
  data_ble[15] = 0;
  data_ble[16] = 0 >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0 >> 8;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0;

  Serial5.write(data_ble, datalength_ble);
}

void g_ID_assignment() {
  switch (M_Selected) { // This is to check which motor was selected
            case 1: // ################### Motor 1
              switch (CtrlMode_Selected) {
                case 1: // Position control
                  switch (Signal_Mode_Selected) {  // Signal mode (constant | sinwave | step)
                    case 1:
                      M1_g_ID = 111;
                    break;
                    case 2:
                      M1_g_ID = 112;
                      m1_sw_bias = sw_bias;
                      m1_sw_amp  = sw_amp;
                      m1_sw_freq = sw_freq;
                    break;
                    case 3:
                      M1_g_ID = 113;
                      m1_step_size       = step_size;
                      m1_step_period     = step_period;
                      m1_step_dutycycle  = step_dutycycle;
                    break;
                  }
                break;
              }
            break;
            case 2: // ################### Motor 2
              switch (CtrlMode_Selected) {
                case 1: // Position control
                  switch (Signal_Mode_Selected) {  // Signal mode (constant | sinwave | step)
                    case 1:
                      M2_g_ID = 211;
                    break;
                    case 2:
                      M2_g_ID = 212;
                      m2_sw_bias = sw_bias;
                      m2_sw_amp  = sw_amp;
                      m2_sw_freq = sw_freq;
                    break;
                    case 3:
                      M2_g_ID = 213;
                      m2_step_size    = step_size;
                      m2_step_period = step_period;
                      m2_step_dutycycle  = step_dutycycle;
                    break;
                  }
                break;
              }
            break;
          }
}


void M1_cdm_assignment() {
  switch (M1_g_ID) {
    // Motor 1
    case 111:
      if (M_active == 1) {
        m1_pos_cmd = pos_cmd;
      }
    break;
    case 112:
      m1_pos_cmd = m1_sw_bias + m1_sw_amp * sin((2 * PI * m1_sw_freq) * (t / 1000000.0));
    break;
    case 113:
      m1_pos_cmd = StepFunction(m1_step_size, m1_step_period, m1_step_dutycycle, t);
    break;
  }
}


void M2_cdm_assignment() {
  switch (M2_g_ID) {
    // Motor 2
    case 211:
      if (M_active == 2) {
        m2_pos_cmd = pos_cmd;
      }
    break;
    case 212:
      m2_pos_cmd = m2_sw_bias + m2_sw_amp * sin((2 * PI * m2_sw_freq) * (t / 1000000.0));
    break;
    case 213:
      m2_pos_cmd = StepFunction(m2_step_size, m2_step_period, m2_step_dutycycle, t);
    break;
  }
}

void cdm_assignment() {
  M1_cdm_assignment();
  M2_cdm_assignment();
}


// int StepFunction(int step_size, int t_high, int t_low, int t) {
//     int period = t_high + t_low;  // Calculate the total period of the signal
//     int mod_time = fmod(t, period);     // Determine the position in the current cycle

//     if (mod_time < t_high) {
//         // If within the high duration, return the step size
//         return step_size;
//     } else {
//         // Otherwise, return 0 for the low duration
//         return 0;
//     }
// }


// double StepFunction(double step_size, double period, double duty_cycle, unsigned long t) {
//   double t_high = period * (duty_cycle / 100); // Calculate the high time based on the duty cycle
//   // int t_low = period - t_high; // Calculate the low time
//   double mod_time = (t / 1000000.0) % period; // Determine the position in the current cycle

//   if (mod_time < t_high) {
//     // If within the high duration, return the step size
//     return step_size;
//   } else {
//     // Otherwise, return 0 for the low duration
//     return 0;
//   }
// }


double StepFunction(double step_size, double period, double duty_cycle, unsigned long t) {
  double t_high = period * (duty_cycle / 100.0); // Calculate the high time based on the duty cycle
  double mod_time = fmod(t / 1000000.0, period); // Determine the position in the current cycle

  if (mod_time < t_high) {
    // If within the high duration, return the step size
    return step_size;
  } else {
    // Otherwise, return 0 for the low duration
    return 0;
  }
}