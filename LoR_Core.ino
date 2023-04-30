/* LORD of ROBOTS - LoR_Core - 202304080054
  This code is designed to control a robot using a PS4 controller, a NeoPixel LED strip, and PWM motor control. The robot can also be controlled via serial communication or PWM as an alternative control methods. The code is organized into sections, each serving a specific purpose.
    1. Libraries: The necessary libraries for the PS4 controller, NeoPixel strip, and PWM motor control are included.
    2. Definitions: IO interfaces, motor pins, PWM configurations, NeoPixel configurations, and joystick control variables are defined.
    3. Setup: The setup() function initializes pin configurations, LED PWM functionalities, the PS4 controller, and serial communication.
    4. Main Loop: The loop() function handles the PS4 controller and serial input, motor output, and sets the color of the NeoPixel strip.
    5. Helper Functions: Several helper functions are included to manage NeoPixel strip patterns and colors, process joystick input, handle slew rate for motor speed ramping, control motor output based on input values, and handle serial control input.
  Overall, the code provides a comprehensive solution for controlling a robot with a PS4 controller and serial communication, while also integrating a NeoPixel LED strip for visual feedback.
 
  Control inputs - LED Indication:
    PS4 - Rainbow LED
    Serial - Green LED
    RC pwm - Blue LED
    none/Stop/standby - Red LED

  Drive configurations:
    Mecanum 
    Standard tank style

*/
#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>
#include <Adafruit_NeoPixel.h>

// version control and major control function settings
String Version = "Base Version : 0.1.1";
bool MecanumDrive_Enabled = false;
bool PWM_Control_Enabled = false;
bool SerialControl_Enabled = true;

// IO Interface Definitions
#define LED_DataPin 12
#define LED_COUNT 36
#define ControllerSelectPin 34
#define MotorEnablePin 13
#define channel1Pin 16
#define channel2Pin 17
#define channel3Pin 21
#define channel4Pin 22


// Motor Pin Definitions
#define motorPin_M1_A 5
#define motorPin_M1_B 14
#define motorPin_M2_A 18
#define motorPin_M2_B 26
#define motorPin_M3_A 23
#define motorPin_M3_B 19
#define motorPin_M4_A 15
#define motorPin_M4_B 33
#define motorPin_M5_A 27
#define motorPin_M5_B 25
#define motorPin_M6_A 32
#define motorPin_M6_B 4
const int motorPins_A[] = { motorPin_M1_A, motorPin_M2_A, motorPin_M3_A, motorPin_M4_A, motorPin_M5_A, motorPin_M6_A };
const int motorPins_B[] = { motorPin_M1_B, motorPin_M2_B, motorPin_M3_B, motorPin_M4_B, motorPin_M5_B, motorPin_M6_B };

// PWM Configuration Definitions
const int Motor_M1_A = 0;
const int Motor_M1_B = 1;
const int Motor_M2_A = 2;
const int Motor_M2_B = 3;
const int Motor_M3_A = 4;
const int Motor_M3_B = 5;
const int Motor_M4_A = 6;
const int Motor_M4_B = 7;
const int Motor_M5_A = 8;
const int Motor_M5_B = 9;
const int Motor_M6_A = 10;
const int Motor_M6_B = 11;
const int MOTOR_PWM_Channel_A[] = { Motor_M1_A, Motor_M2_A, Motor_M3_A, Motor_M4_A, Motor_M5_A, Motor_M6_A };
const int MOTOR_PWM_Channel_B[] = { Motor_M1_B, Motor_M2_B, Motor_M3_B, Motor_M4_B, Motor_M5_B, Motor_M6_B };
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;

// NeoPixel Configurations
Adafruit_NeoPixel strip(LED_COUNT, LED_DataPin, NEO_GRB + NEO_KHZ800);
const uint32_t RED = strip.Color(255, 0, 0, 0);
const uint32_t GREEN = strip.Color(0, 255, 0, 0);
const uint32_t BLUE = strip.Color(0, 0, 255, 0);
const uint32_t WHITE = strip.Color(0, 0, 0, 255);
const uint32_t PURPLE = strip.Color(255, 0, 255, 0);
const uint32_t CYAN = strip.Color(0, 255, 255, 0);
const uint32_t YELLOW = strip.Color(255, 255, 0, 0);


// Joystick control variables
const int DEAD_BAND = 20;
const float TURN_RATE = 1.5;

// Motor speed limits and starting speed
const int MAX_SPEED = 255;
const int MIN_SPEED = -255;
const int MIN_STARTING_SPEED = 140;
const int STOP = 0;
const int SerialControl_SPEED = 110;
bool INVERT = true;

// Slew rate for ramping motor speed
const int SLEW_RATE_MS = 20;

// Define variables to store the PWM signal values
int PWM_Input_LY = 0;
int PWM_Input_LX = 0;
int PWM_Input_RY = 0;
int PWM_Input_RX = 0;

// Define variables to store the PWM signal values
volatile int channel1Value = 0;
volatile int channel2Value = 0;
volatile int channel3Value = 0;
volatile int channel4Value = 0;

// Define variables to store the time of the last pulse on each PWM channel
volatile unsigned long channel1LastRising = 0;
volatile unsigned long channel1LastFalling = 0;
volatile unsigned long channel2LastRising = 0;
volatile unsigned long channel2LastFalling = 0;
volatile unsigned long channel3LastRising = 0;
volatile unsigned long channel3LastFalling = 0;
volatile unsigned long channel4LastRising = 0;
volatile unsigned long channel4LastFalling = 0;

// Define the interrupt service routines for each PWM channel
void IRAM_ATTR channel1ISR() {
  unsigned long now = micros();
  if (digitalRead(channel1Pin) == HIGH) {
    channel1LastRising = now;
  } else {
    channel1Value = now - channel1LastRising;
    channel1LastFalling = now;
  }
}

void IRAM_ATTR channel2ISR() {
  unsigned long now = micros();
  if (digitalRead(channel2Pin) == HIGH) {
    channel2LastRising = now;
  } else {
    channel2Value = now - channel2LastRising;
    channel2LastFalling = now;
  }
}

void IRAM_ATTR channel3ISR() {
  unsigned long now = micros();
  if (digitalRead(channel3Pin) == HIGH) {
    channel3LastRising = now;
  } else {
    channel3Value = now - channel3LastRising;
    channel3LastFalling = now;
  }
}

void IRAM_ATTR channel4ISR() {
  unsigned long now = micros();
  if (digitalRead(channel4Pin) == HIGH) {
    channel4LastRising = now;
  } else {
    channel4Value = now - channel4LastRising;
    channel4LastFalling = now;
  }
}

// Set up pins, LED PWM functionalities and begin PS4 controller, Serial and Serial2 communication
void setup() {
  // Set up the pins
  pinMode(LED_DataPin, OUTPUT);
  pinMode(ControllerSelectPin, INPUT_PULLUP);
  pinMode(MotorEnablePin, OUTPUT);

  for (int i = 0; i < 6; i++) {
    pinMode(motorPins_A[i], OUTPUT);
    pinMode(motorPins_B[i], OUTPUT);
    digitalWrite(motorPins_A[i], 0);
    digitalWrite(motorPins_B[i], 0);
  }

  // configureation of aux pins for pwm control (alternative control method)
  if (PWM_Control_Enabled) {
    pinMode(channel1Pin, INPUT_PULLDOWN);
    pinMode(channel2Pin, INPUT_PULLDOWN);
    pinMode(channel3Pin, INPUT_PULLDOWN);
    pinMode(channel4Pin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(channel1Pin), channel1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel2Pin), channel2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel3Pin), channel3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel4Pin), channel4ISR, CHANGE);
  }

  // output preset bias
  digitalWrite(LED_DataPin, 0);
  digitalWrite(MotorEnablePin, 1);

  // configure LED PWM functionalitites
  for (int i = 0; i < 6; i++) {
    ledcSetup(MOTOR_PWM_Channel_A[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR_PWM_Channel_B[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(motorPins_A[i], MOTOR_PWM_Channel_A[i]);
    ledcAttachPin(motorPins_B[i], MOTOR_PWM_Channel_B[i]);
  }

  // Neopixels Configuration
  strip.begin();            // INITIALIZE NeoPixel strip object
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)

  // PS4 controller configuration (Target mac address saved on the controller)
    // PS4.begin("a8:47:4a:da:7d:9c");  // Black db PS4 
    // PS4.begin("a8:47:4a:f2:e0:5e");  // white camio db PS4 
    // PS4.begin("40:99:22:89:c5:20");  // Red db PS4 
    // PS4.begin("20:04:06:1a:00:b2");  // black JD PS4
    PS4.begin("30:11:aa:01:13:02");  // gold JD PS4

  // Serial comms configurations (USB for debug messages)
  Serial.begin(115200);  // USB Serial

  //Serial Contol port configuration as input for control from modules like Camron. (Alternative control method)
  if (SerialControl_Enabled) Serial2.begin(115200, SERIAL_8N1, 16, 17);  //  secondary serial to communicate with the Camron module

  delay(2000);

  //debug messages
  Serial.print("Serial Control: ");
  if (SerialControl_Enabled) Serial.println("Enabled");
  else Serial.println("Disabled");

  Serial.print("PWM Control: ");
  if (PWM_Control_Enabled) Serial.println("Enabled");
  else Serial.println("Disabled");

  Serial.print("Mecanum Drive: ");
  if (MecanumDrive_Enabled) Serial.println("Enabled");
  else Serial.println("Disabled");

  Serial.println("Core System Ready! " + Version);
}

void loop() {
  // Main loop to handle PS4 controller and serial input
  //PS4 Control
  if (PS4.isConnected()) {
    PS4.setLed(0, 255, 0);
    PS4.sendToController();
    NeoPixel_Rainbow();                                           // LED Display
    Motion_Control(PS4.LStickY(), PS4.LStickX(), PS4.RStickX());  // Joystick control
    delay(5);
  //Serial Control  
  } else if (SerialControl()) {
    NeoPixel_SetColour(GREEN);
  //PWM Control    
  } else if (PWM_Control()) {
    NeoPixel_SetColour(BLUE);
    Motion_Control(PWM_Input_LY, PWM_Input_LX, PWM_Input_RX);  // RC control
  //Stop/Standby
  } else {
    NeoPixel_SetColour(RED);
    Motor_STOP();
  }
  //Set motor output values
  Motor_Control();
}

// Rainbow pattern for NeoPixel strip
long firstPixelHue = 0;
void NeoPixel_Rainbow() {
  strip.rainbow(firstPixelHue);
  strip.show();  // Update strip with new contents
  firstPixelHue += 256;
  if (firstPixelHue >= 5 * 65536) firstPixelHue = 0;
}

// Set a specific color for the entire NeoPixel strip
void NeoPixel_SetColour(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    strip.show();                                // Update strip with new contents
  }
}

// Process joystick input and calculate motor speeds - Mecanum control
int Motor_FrontLeft_SetValue, Motor_FrontRight_SetValue, Motor_BackLeft_SetValue, Motor_BackRight_SetValue = 0;
void Motion_Control(int LY_Axis, int LX_Axis, int RX_Axis) {

  int FrontLeft_TargetValue, FrontRight_TargetValue, BackLeft_TargetValue, BackRight_TargetValue = 0;
  int ForwardBackward_Axis = LY_Axis;
  int StrafeLeftRight_Axis = LX_Axis;
  int TurnLeftRight_Axis = -RX_Axis;

  //Set deadband
  if (abs(ForwardBackward_Axis) < DEAD_BAND) ForwardBackward_Axis = 0;
  if (abs(StrafeLeftRight_Axis) < DEAD_BAND) StrafeLeftRight_Axis = 0;
  if (abs(TurnLeftRight_Axis) < DEAD_BAND) TurnLeftRight_Axis = 0;

  //Calculate strafe values
  FrontLeft_TargetValue = -ForwardBackward_Axis + (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  BackLeft_TargetValue = -ForwardBackward_Axis - (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  FrontRight_TargetValue = ForwardBackward_Axis + (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  BackRight_TargetValue = ForwardBackward_Axis - (StrafeLeftRight_Axis * MecanumDrive_Enabled);

  //calculate rotation values
  if (abs(TurnLeftRight_Axis) > DEAD_BAND) {
    FrontLeft_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    BackLeft_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    FrontRight_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    BackRight_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
  }

  //constrain to joystick range
  FrontLeft_TargetValue = constrain(FrontLeft_TargetValue, -127, 127);
  BackLeft_TargetValue = constrain(BackLeft_TargetValue, -127, 127);
  FrontRight_TargetValue = constrain(FrontRight_TargetValue, -127, 127);
  BackRight_TargetValue = constrain(BackRight_TargetValue, -127, 127);

  //set motor speed through slew rate function
  Motor_FrontLeft_SetValue = SlewRateFunction(FrontLeft_TargetValue, Motor_FrontLeft_SetValue);
  Motor_FrontRight_SetValue = SlewRateFunction(FrontRight_TargetValue, Motor_FrontRight_SetValue);
  Motor_BackLeft_SetValue = SlewRateFunction(BackLeft_TargetValue, Motor_BackLeft_SetValue);
  Motor_BackRight_SetValue = SlewRateFunction(BackRight_TargetValue, Motor_BackRight_SetValue);
}

// Function to handle slew rate for motor speed ramping
int SlewRateFunction(int Input_Target, int Input_Current) {
  int speedDiff = Input_Target - Input_Current;
  if (speedDiff > 0) Input_Current += min(speedDiff, SLEW_RATE_MS);
  else if (speedDiff < 0) Input_Current -= min(-speedDiff, SLEW_RATE_MS);
  constrain(Input_Current, -127, 127);
  return Input_Current;
}

void Motor_Control() {
  Set_Motor_Output(Motor_FrontLeft_SetValue, Motor_M1_A, Motor_M1_B);
  Set_Motor_Output(Motor_BackLeft_SetValue, Motor_M2_A, Motor_M2_B);
  Set_Motor_Output(Motor_FrontRight_SetValue, Motor_M5_A, Motor_M5_B);
  Set_Motor_Output(Motor_BackRight_SetValue, Motor_M6_A, Motor_M6_B);
}

void Motor_STOP() {
  Set_Motor_Output(STOP, Motor_M1_A, Motor_M1_B);
  Set_Motor_Output(STOP, Motor_M2_A, Motor_M2_B);
  Set_Motor_Output(STOP, Motor_M5_A, Motor_M5_B);
  Set_Motor_Output(STOP, Motor_M6_A, Motor_M6_B);
}


// Function to control motor output based on input values
void Set_Motor_Output(int Output, int Motor_ChA, int Motor_ChB) {
  if (INVERT) Output = -Output;

  Output = constrain(Output, -127, 127);

  int Mapped_Value = map(abs(Output), 0, 127, MIN_STARTING_SPEED, MAX_SPEED);
  int A, B = 0;
  if (Output < -DEAD_BAND) {  // Rotate Clockwise
    A = 0;
    B = Mapped_Value;
  } else if (Output > DEAD_BAND) {  // Rotate Counter-Clockwise
    A = Mapped_Value;
    B = 0;
  } else {  // Rotation Stop
    A = STOP;
    B = STOP;
  }
  ledcWrite(Motor_ChA, A);  //send to motor control pins
  ledcWrite(Motor_ChB, B);
}



// Function to handle serial control input
// Define variables to store the current motor speeds
bool STOP_FLAG = true;
long TIME_OUT = 0;
float SPEED_Adjustment = 0.75;
int Serial_Input_L = 0;
int Serial_Input_R = 0;
boolean SerialControl() {
  if (!SerialControl_Enabled) return false;

  if (Serial2.available()) {
    String request = Serial2.readStringUntil('\n');  // ensure entire packet is recieved

    Serial.print("READ ---- ");
    Serial.println(request);  //reply

    while (Serial2.available()) { Serial2.read(); }  // clear/dump buffer

    STOP_FLAG = false;

    // Check the received string and change the robot's direction accordingly
    if (request.indexOf("Forward") != -1) {
      Serial_Input_L = int (-SerialControl_SPEED * SPEED_Adjustment);
      Serial_Input_R = int (SerialControl_SPEED * SPEED_Adjustment);
      TIME_OUT = millis() + 200;
    } else if (request.indexOf("Backward") != -1) {
      Serial_Input_L = int (SerialControl_SPEED * SPEED_Adjustment);
      Serial_Input_R = int (-SerialControl_SPEED * SPEED_Adjustment);
      TIME_OUT = millis() + 200;
    } else if (request.indexOf("Left") != -1) {
      Serial_Input_L = SerialControl_SPEED;
      Serial_Input_R = SerialControl_SPEED;
      TIME_OUT = millis() + 200;
    } else if (request.indexOf("Right") != -1) {
      Serial_Input_L = -SerialControl_SPEED;
      Serial_Input_R = -SerialControl_SPEED;
      TIME_OUT = millis() + 200;
    } else if (request.indexOf("Stop") != -1) {
      Serial_Input_L = STOP;
      Serial_Input_R = STOP;
      STOP_FLAG = true;
    } else if (millis() > TIME_OUT) {
      Serial_Input_L = STOP;
      Serial_Input_R = STOP;
      STOP_FLAG = true;
    } else {
      STOP_FLAG = true;
    }
  }
  if (millis() > TIME_OUT && TIME_OUT != 0) {
    Serial_Input_L = STOP;
    Serial_Input_R = STOP;
    STOP_FLAG = true;
  }

  Motor_FrontLeft_SetValue = Serial_Input_L;
  Motor_BackLeft_SetValue = Serial_Input_L;
  Motor_FrontRight_SetValue = Serial_Input_R;
  Motor_BackRight_SetValue = Serial_Input_R;

  if (STOP_FLAG) return false;
  else return true;
}


boolean PWM_Control() {
  if (!PWM_Control_Enabled) return false;
  PWM_Input_LY = int(map(channel1Value, 1000, 2000, -127, 127) * 2);
  PWM_Input_LX = int(map(channel2Value, 1000, 2000, -127, 127) * 2);
  PWM_Input_RY = int(map(channel3Value, 1000, 2000, -127, 127));
  PWM_Input_RX = int(map(channel4Value, 1000, 2000, -127, 127));
  if (PWM_Input_LY + PWM_Input_LX + PWM_Input_RX + PWM_Input_RY < (-128 * 4)) return false;  // no signal threshold

  return true;
}
