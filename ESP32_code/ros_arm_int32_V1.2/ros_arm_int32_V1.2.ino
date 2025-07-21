#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>

// V1.1 connecting the controlls to the feedbaack

//* £££££££££££££££££££££££££ Parameters ££££££££££££££££££££££££££££££££££££££££££££££££££££

const int DEADBAND = 100;  // Encoder ticks tolerance
const float rad_to_tick_scale[6] = {110000/3.14, 110000/3.14, 110000/3.14, 110000/3.14, 110000/3.14, 110000/3.14};

unsigned long lastDisplayUpdate = 0;
unsigned long UdapteDisplayEvery = 100;


//*===================== Micro-ros elemints ==============================
rcl_subscription_t subscriber;
std_msgs__msg__Int64 msg;

rcl_subscription_t config_subscriber;
std_msgs__msg__Int64 config_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//* &&&&&&&&&&&&&&&&&&&&&& Hardware setup &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//* OLED Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define SSD1306_I2C_ADDRESS 0x3c


//* motor setup
// Motor A joint 5 connections
int in1 = 19;
int in2 = 18;

// Motor Channels (adjust per wiring)
int motorChannels[5][3] = {
    {0, 1, 2},  // Joint 1: PWM, IN1, IN2
    {5, 4, 3},  // Joint 2
    {6, 7, 8},  // Joint 3
    {11, 10, 9},  // Joint 4
    {12, 13, 14},  // Joint 5
  };

//* PCA96685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int PWM_MIN = 2600;
const int PWM_MAX = 4095;



//* Encoder setup    
int encoderPins[6][2] = {{36, 39}, {34, 35}, {32, 33}, {25, 26}, {27, 14}, {13, 23}};  // Encoder Pins  //! dont use GPIO19: USB D+ (Data Positive), GPIO20: USB D- (Data Negative), GPIO43: USB-JTAG D-, GPIO44: USB-JTAG D+, GPIO0: Boot mode select (HIGH = Normal boot, LOW = Download mode), GPIO45: VSPI CS (Strapping pin for boot voltage), GPIO46: Input-only but used for boot strapping
// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder* encoders[6]; // Array of encoder objects

// @brief The interrupt service routine will be called on any change of one of the input signals.
void IRAM_ATTR isrEncoder0()
{
  encoders[0]->tick(); // just call tick() to check the state.
}
void IRAM_ATTR isrEncoder1() { encoders[1]->tick(); }
void IRAM_ATTR isrEncoder2() { encoders[2]->tick(); }
void IRAM_ATTR isrEncoder3() { encoders[3]->tick(); }
void IRAM_ATTR isrEncoder4() { encoders[4]->tick(); }
void IRAM_ATTR isrEncoder5() { encoders[5]->tick(); }

void (*isrFunctions[6])() = {isrEncoder0, isrEncoder1, isrEncoder2, isrEncoder3, isrEncoder4, isrEncoder5};




//* PID setup
// PID Parameters
double Kp[6] = {0.05, 0.000000005, 0.05, 0.05, 0.05, 0.05 };  //TODO Start with these values and tune
double Ki[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
double Kd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// PID Variables
double setpoint[6] = {0};
double input[6] = {0};
double output[6] = {0};

// PID Object
PID myPID[6] = {
    PID(&input[0], &output[0], &setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT),
    PID(&input[1], &output[1], &setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT),
    PID(&input[2], &output[2], &setpoint[2], Kp[2], Ki[2], Kd[2], DIRECT),
    PID(&input[3], &output[3], &setpoint[3], Kp[3], Ki[3], Kd[3], DIRECT),
    PID(&input[4], &output[4], &setpoint[4], Kp[4], Ki[4], Kd[4], DIRECT),
    PID(&input[5], &output[5], &setpoint[5], Kp[5], Ki[5], Kd[5], DIRECT),
  };


//* ^^^^^^^^^^^^^^^^^^^^^^^^^ Functions ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void wrist_controll(){
  // compute input
  in_encoder1 = encoders[1]->getPosition();
  in_encoder2 = encoders[2]->getPosition();
  
  if (in_encoder1 == in_encoder2){
    input[1] = 10000; // mid point is 0 degre in rotation
    input[2] = in_encoder2;
  }else if (in_encoder1 < in_encoder2){
    input[1] = 10000 + in_encoder2 - in_encoder1;
    input[2] = (in_encoder1 + (in_encoder2 - in_encoder1)/2);
  }else {
    input[1] = 10000 - (in_encoder1 - in_encoder2;)
    input[2] = (in_encoder1 - (in_encoder1 - in_encoder2)/2);
  }

  // compute setpoint  ** not needed
  // setpoint_1 = setpoint[1];
  // setpoint_2 = setpoint[2];


  // linear move
  myPID[2].Compute();
  int in1 = (output[2] > 0) ? 4096 : 0;
  int in2 = (output[2] > 0) ? 0 : 4096;
  // first mottor
  pwm.setPWM(motorChannels[1][1], in1, 4096 - in1);  // IN1
  pwm.setPWM(motorChannels[1][2], in2, 4096 - in2);  // IN2
  pwm.setPWM(motorChannels[1][0], 0, pwmVal);        // PWM
  // second mottor same move with inversed direction
  pwm.setPWM(motorChannels[2][2], in1, 4096 - in1);  // IN1
  pwm.setPWM(motorChannels[2][1], in2, 4096 - in2);  // IN2
  pwm.setPWM(motorChannels[2][0], 0, pwmVal);        // PWM

  // rotational move
  myPID[1].Compute();
  int in1 = (output[1] > 0) ? 4096 : 0;
  int in2 = (output[1] > 0) ? 0 : 4096;
  // first mottor
  pwm.setPWM(motorChannels[1][1], in1, 4096 - in1);  // IN1
  pwm.setPWM(motorChannels[1][2], in2, 4096 - in2);  // IN2
  pwm.setPWM(motorChannels[1][0], 0, pwmVal);        // PWM
  // second mottor same move 
  pwm.setPWM(motorChannels[2][1], in1, 4096 - in1);  // IN1
  pwm.setPWM(motorChannels[2][2], in2, 4096 - in2);  // IN2
  pwm.setPWM(motorChannels[2][0], 0, pwmVal);        // PWM
}

void controlMotor(int joint) {
    input[joint] = encoders[joint]->getPosition();
    myPID[joint].Compute();
  
    if (abs(input[joint] - setpoint[joint]) > DEADBAND) {
        int pwmVal = constrain(abs(output[joint]), PWM_MIN, PWM_MAX);

        if(joint == 5) {
            // L298N Control for Joint 5 using GPIOs 11,12,13
            digitalWrite(in2, (output[joint] > 0) ? HIGH : LOW);  // IN1
            digitalWrite(in1, (output[joint] > 0) ? LOW : HIGH);  // IN2
            pwm.setPWM(15, 0, pwmVal);        // PWM
        }else if (joint == 1 || joint == 2){
            wrist_controll();
        } else {
            // PCA9685 Control for other joints
            int in1 = (output[joint] > 0) ? 4096 : 0;
            int in2 = (output[joint] > 0) ? 0 : 4096;
            pwm.setPWM(motorChannels[joint][1], in1, 4096 - in1);  // IN1
            pwm.setPWM(motorChannels[joint][2], in2, 4096 - in2);  // IN2
            pwm.setPWM(motorChannels[joint][0], 0, pwmVal);        // PWM
        }
    } else {
        if(joint == 5) {
            // Brake for Joint 5
            digitalWrite(11, LOW);
            digitalWrite(12, LOW);
            analogWrite(13, 0);
        } else {
            // PCA9685 Brake for other joints
            pwm.setPWM(motorChannels[joint][0], 0, 4096);
        }
    }
}





//*+++++++++++++++++++++++++ Callbacks ++++++++++++++++++++++++++++++++++++++++ 

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
  int64_t data = msg->data;
    
    // // Verify the message starts with '1' (19-digit number)
    // if (data < 1000000000000000000 || data >= 2000000000000000000) {
    //     display.clearDisplay();
    //     display.setCursor(0, 0);
    //     display.println("INVALID MSG");
    //     display.display();
    //     return;
    // }
    
    // Remove the leading '1'
    data %= 1000000000000000000LL;  // Remove first digit
    
    // Extract each 3-digit group (right to left)
    int values[6];
    for (int i = 5; i >= 0; i--) {
        values[i] = data % 1000;  // Get last 3 digits
        data /= 1000;              // Remove those digits
    }
    
    // Update setpoints
    // for (int joint = 0; joint < 6; joint++) {
    //     // Convert to position: (value/100.0) - 3.14
    //     setpoint[joint] = ((values[joint] * 0.01f) - 3.14f)*rad_to_tick_scale ;
    // }
    
    setpoint[0] = ((values[5] * 0.01f) - 3.14f)*rad_to_tick_scale[0] ;
    setpoint[1] = ((values[4] * 0.01f) - 3.14f)*rad_to_tick_scale[1] ;
    setpoint[2] = ((values[3] * 0.01f) - 3.14f)*rad_to_tick_scale[2] ;
    setpoint[3] = ((values[2] * 0.01f) - 3.14f)*rad_to_tick_scale[3] ;
    setpoint[4] = ((values[1] * 0.01f) - 3.14f)*rad_to_tick_scale[4] ;
    setpoint[5] = ((values[0] * 0.01f) - 3.14f)*rad_to_tick_scale[5] ;

    
    Optional display update
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("J0: %.2f rad", setpoint[0]);
    display.println();
    display.printf("J1: %.2f rad", setpoint[1]);
    display.println();
    display.printf("J2: %.2f rad", setpoint[2]);
    display.println();
    display.printf("J3: %.2f rad", setpoint[3]);
    display.println();
    display.printf("J4: %.2f rad", setpoint[4]);
    display.println();
    display.printf("J5: %.2f rad", setpoint[5]);
    display.println();
    display.display();
}



void config_callback(const void * msgin)
{  
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
  int32_t raw_value = msg->data;
    
  // Extract components with proper digit grouping
    int joint = raw_value / 100000000;       // 1 digit (10^8)
    double kp = (raw_value % 100000000) / 1000000.0;  // Next 3 digits 00.0
    double ki = (raw_value % 100000) / 1000.0;      // Next 2 digits 00.0
    double kd = (raw_value % 100) / 1.0;          // Last 3 digits 00
    
    // Validate joint
    if (joint < 0 || joint >= 6){
      display.setCursor(0, 0);
      display.clearDisplay();
      display.println("WRONG VALUE");
      display.display();
      return;
    }
    
    // Update PID
    Kp[joint] = kp;
    Ki[joint] = ki;
    Kd[joint] = kd;
    myPID[joint].SetTunings(kp, ki, kd);
    
    // Display update
    display.setCursor(0, 0);
    display.clearDisplay();
    display.println(msg->data);
    display.printf("J%d: P%.1f I%.1f D%.1f", joint, kp, ki, kd);
    display.display();

}




//* %%%%%%%%%%%%%%%%%%%%%%%%% Main Voids %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_ESP32_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "joint_states_int64"));

  RCCHECK(rclc_subscription_init_default(
    &config_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "config_int64"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &config_subscriber, &config_msg, &config_callback, ON_NEW_DATA));

  //* Initialize I2C
  Wire.begin(21,22); // SDA, SCL

  //* Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.println("Ready");
  display.display();


  //* Initialize the motor
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  
  //* Initialize the PCA96685 pins to off
  pwm.begin();
  pwm.setPWMFreq(20000);
  for (int k = 0; k < 16; k++){
        pwm.setPWM(k, 0, 4096); // turns pin fully off
    }


  //* Initialize the Encoders  
  for (int i = 0; i < 6; i++){
    encoders[i] = new RotaryEncoder(encoderPins[i][0], encoderPins[i][1], RotaryEncoder::LatchMode::TWO03);
    attachInterrupt(encoderPins[i][0], isrFunctions[i], CHANGE);
    attachInterrupt(encoderPins[i][1], isrFunctions[i], CHANGE);
  }


  //* Initialize PID
  for (int i = 0; i < 6; i++) {
    myPID[i].SetMode(AUTOMATIC);
    myPID[i].SetOutputLimits(-PWM_MAX, PWM_MAX);
    myPID[i].SetSampleTime(1);
  }

}

void loop() {
  static unsigned long lastUpdate = millis();
    
  // Run PID control at 50Hz
  if (millis() - lastUpdate >= 20) {
    for (int i = 0; i < 6; i++) {
      controlMotor(i);
    }
    lastUpdate = millis();
  }

  // Display update
  if (millis() - lastDisplayUpdate > UdapteDisplayEvery) { 
        display.clearDisplay();
        display.setCursor(0, 0); // Reset cursor to top-left corner
        for (int i = 0; i < 6; i++) {
        display.printf("Pos : %ld\n", encoders[i]->getPosition());
        }
        display.display();
        lastDisplayUpdate = millis();
      }

  // Handle micro-ROS messages
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
