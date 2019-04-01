#include <Wire.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include<geometry_msgs/Vector3.h>

// steering motor
#define PWM_PIN_A1  6   // counter-clockwise
#define PWM_PIN_B1  5   // clockwise

//the interrupt is avaliable in pin 2 and pin 3 arduino uno
#define INT0_PIN  2
#define INT1_PIN  3

#define INT0      0
#define INT1      1

//in the master wire.begin stated as device number 8
#define DEVICE_NUMBER 8
#define BYTES 1
//baud rate for rosserial
#define BAUD_RATE  57600

//baud rate for serial communication of arduino- incase of using arduino serial monitor
#define Serial_baud_rate 9600


// driving motor
#define PWM_PIN_A2  9   // counter-clockwise
#define PWM_PIN_B2  10  // clockwise

// maximum and minimum speed of the driving motor
#define MAX_PWM 255
#define MIN_PWM 0

//customize  ros handle
#define MAX_PUBLISHERS    2
#define MAX_SUBSCRIBERS   2
#define IN_BUFFER_SIZE    128
#define OUT_BUFFER_SIZE   128

// MegaMoto enable pins
#define ENABLE_PIN1 8     // steering
#define ENABLE_PIN2 12    // driving

// duty cycle range: 0 ~ 255
// constant speed settings
#define INIT_PWM_DRIVING  60

// speed change step: percentage
#define SPEED_CHANGE_STEP   5

// delay before stopping steering motor after making change
#define DELAY_AFTER_STEERING   100

// steering encoder min/max count
// we remap the raw encoder tick count to this range
#define MIN_STEERING_ENCODER_COUNT  -100  // right
#define MAX_STEERING_ENCODER_COUNT  100    // left

// this is not symetric due to the possible misalignment of the steeering wheel
#define MIN_ADJ_STEERING_ENCODER_COUNT  -100  // right
#define MAX_ADJ_STEERING_ENCODER_COUNT  100    // left

// min-max pulse from the steering encoder
#define MIN_PULSE_STEERING_ENCODER_COUNT  -561
#define MAX_PULSE_STEERING_ENCODER_COUNT  561

// tolerance margin for centering
#define CENTERING_MARGIN    7

// we want to start the steering quickly first and slow down later not to overshoot
#define INIT_PWM_CYCLE_FOR_CENTERING    80
#define END_PWM_CYCLE_FOR_CENTERING     50
#define DELAY_BETWEEN_INIT_END_PWM_CYCLE_FOR_CENTERING  50  // ms

// main loop delay
#define MAIN_LOOP_DELAY   25

/////////////////////////////////////////////////////////////////////////////////////

//limit motor action based on counts prevent gearbox gears wear
int limit_extreme_left_count = 100;
int limit_left_count = 100;
int limit_right_count = -100;
int limit_extreme_right_count = -100;

// steering motor speed
int steer_speed = 100;

//variable for incremental encoder counter values
int steering_encoder_count = 0; // -100 ~ 100
// pulse count from the steering encoder
int tick_count = 0;             // -561 ~ 561

int max_mapped_speed = 100;
int min_mapped_speed = 0;

//mapping the driving motor speed informat of 0 to 100
int pwm_duty_cycle_drive = map(INIT_PWM_DRIVING, MIN_PWM, MAX_PWM, 0, 100);

int pwm_duty_cycle_steer = map(steer_speed, MIN_PWM, MAX_PWM, 0, 100);

// for joystick processing command and state is for saving previous command
int state;
int command;

//vector3 is a float of x,y,z values , used to publish the driving and steering encoder value over ROS topic named pos. x is speed- y is steer- z not used.
geometry_msgs::Vector3 sent;
ros::Publisher pos("encoder_pulse", &sent);

///////////////////////////////////////////////////motor driver////////////////////////////////////////
int pwmRemap(int duty_cycle) {
  return map(duty_cycle, 0, 100, MIN_PWM, MAX_PWM);
}

void stopSteeringMotor() {
  analogWrite(PWM_PIN_B1, 0);
  analogWrite(PWM_PIN_A1, 0);
}

void stopDrivingMotor() {
  analogWrite(PWM_PIN_B2, 0);
  analogWrite(PWM_PIN_A2, 0);
}

void rightSteeringMotor(int duty_cycle) {
  analogWrite(PWM_PIN_B1, 0);
  analogWrite(PWM_PIN_A1, pwmRemap(duty_cycle));
}

void leftSteeringMotor(int duty_cycle) {
  analogWrite(PWM_PIN_B1, pwmRemap(duty_cycle));
  analogWrite(PWM_PIN_A1, 0);
}

void forwardDrivingMotor(int duty_cycle) {
  analogWrite(PWM_PIN_B2, 0);
  analogWrite(PWM_PIN_A2, pwmRemap(duty_cycle));
}

void backwardDrivingMotor(int duty_cycle) {
  analogWrite(PWM_PIN_B2, pwmRemap(duty_cycle));
  analogWrite(PWM_PIN_A2, 0);
}

// Creation of a NodeHandle
// ros::NodeHandle_<HardwareType, MAX_PUBLISHERS=25, MAX_SUBSCRIBERS=25, IN_BUFFER_SIZE=512, OUT_BUFFER_SIZE=512> nh;
//ros::NodeHandle_<ArduinoHardware, MAX_PUBLISHERS, MAX_SUBSCRIBERS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> nh;
ros::NodeHandle nh;

//this is the subscriber callback ,that subscriber to topic named Control and then process command to move and steer the vehicle
void manualCb( const std_msgs::UInt16& msg) {
  processCommand (msg.data);
}

/////////////////////////////////////////////////// Joy processing////////////////////////////////////////

//this function process the command recieved from Control ROS topic
//0 is stop , 8 is forward,2 is backward,6 is right,4 is left , 3 is extreme right ,1 is extreme left,* reset , +/- for increase /decrease speed
void processCommand(int command) {
  switch (command)
  {
    case 0: // stop
      stopSteeringMotor();
      stopDrivingMotor();
      state = command;
      break;
    case '1': // extreme left
      if (steering_encoder_count > limit_extreme_left_count) {
        stopSteeringMotor();
      }
      else {
        leftSteeringMotor(pwm_duty_cycle_steer);
        delay(DELAY_AFTER_STEERING);
        stopSteeringMotor();
      }
      state = command;
      break;
    case '4': //left
      if (steering_encoder_count > limit_left_count) {
        stopSteeringMotor();
      }
      else {
        leftSteeringMotor(pwm_duty_cycle_steer);
        delay(DELAY_AFTER_STEERING);
        stopSteeringMotor();
      }
      state = command;
      break;

    case '3'://extreme right
      if (steering_encoder_count < limit_extreme_right_count) {
        stopSteeringMotor();
      }
      else {
        rightSteeringMotor(pwm_duty_cycle_steer);
        delay(DELAY_AFTER_STEERING);
        stopSteeringMotor();

      }
      state = command;
      break;

    case '6'://right
      if (steering_encoder_count < limit_right_count) {
        stopSteeringMotor();
      }
      else {
        rightSteeringMotor(pwm_duty_cycle_steer);
        delay(DELAY_AFTER_STEERING);
        stopSteeringMotor();
      }
      state = command;
      break;

    case '8'://forward
      forwardDrivingMotor(pwm_duty_cycle_drive);
      state = command;
      break;

    case '2'://backward
      backwardDrivingMotor(pwm_duty_cycle_drive);
      state = command;
      break;

    case '+'://increase speed
      if (state == '8')
      {
        pwm_duty_cycle_drive += SPEED_CHANGE_STEP;
        if (pwm_duty_cycle_drive > max_mapped_speed)
        {
          pwm_duty_cycle_drive = max_mapped_speed;
        }
        command = '8';
      } else {
        command = state;
      }
      break;

    case '-'://decrease speed
      if (state == '8')
      {
        pwm_duty_cycle_drive -= SPEED_CHANGE_STEP;
      }
      if (pwm_duty_cycle_drive < min_mapped_speed )
      {
        pwm_duty_cycle_drive = min_mapped_speed;
      }
      command = state;
      break;

    case '*':
      recenter();
      state = command;
      break;
  }

  if (steering_encoder_count > MAX_ADJ_STEERING_ENCODER_COUNT
      || steering_encoder_count < MIN_ADJ_STEERING_ENCODER_COUNT) {
    stopSteeringMotor();
  }
}
///////////////////////////////////////////////////Recenter steering wheels////////////////////////////////////////

//this function to set wheels back to center
void recenter() {
  char steer;
  if ( steering_encoder_count < 0 && steering_encoder_count < -CENTERING_MARGIN)
    steer = 'R';

  if (steering_encoder_count > 0 && steering_encoder_count > CENTERING_MARGIN)
    steer = 'L';

  if (steering_encoder_count > -CENTERING_MARGIN && steering_encoder_count < CENTERING_MARGIN)
    steer = 'S';

  while (steer != 'S') {

    if (steer == 'L') { 
      rightSteeringMotor(INIT_PWM_CYCLE_FOR_CENTERING);
      delay(DELAY_BETWEEN_INIT_END_PWM_CYCLE_FOR_CENTERING);
      rightSteeringMotor(END_PWM_CYCLE_FOR_CENTERING);
      break;
    }
    if (steer == 'R') { 
      leftSteeringMotor(INIT_PWM_CYCLE_FOR_CENTERING);
      delay(DELAY_BETWEEN_INIT_END_PWM_CYCLE_FOR_CENTERING);
      leftSteeringMotor(END_PWM_CYCLE_FOR_CENTERING);
      break;
    }
  }
  stopSteeringMotor();
}

ros::Subscriber<std_msgs::UInt16>  sub("vehicle_control", &manualCb);

///////////////////////////////////////////////////happens once ////////////////////////////////////////

void setup() {
  Wire.begin();
  Wire.onReceive(receiveEvent);
  Serial.begin(Serial_baud_rate);
  pinMode(INT0_PIN, INPUT);           // set pin to input
  pinMode(INT1_PIN, INPUT);           // set pin to input

  digitalWrite(INT0_PIN, HIGH);
  digitalWrite(INT1_PIN, HIGH);
  attachInterrupt(INT0, readChannelA, RISING);
  attachInterrupt(INT1, readChannelB, RISING);

  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  pinMode(PWM_PIN_A1, OUTPUT);
  pinMode(PWM_PIN_B1, OUTPUT);
  pinMode(PWM_PIN_A2, OUTPUT);
  pinMode(PWM_PIN_B2, OUTPUT);
  digitalWrite(ENABLE_PIN1, LOW);
  digitalWrite(ENABLE_PIN2, LOW);

  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pos);
}

///////////////////////////////////////////////////Loop////////////////////////////////////////

void loop() {
  digitalWrite(ENABLE_PIN1, HIGH);
  digitalWrite(ENABLE_PIN2, HIGH);
  steering_encoder_count = map(tick_count,
                               MIN_PULSE_STEERING_ENCODER_COUNT,
                               MAX_PULSE_STEERING_ENCODER_COUNT,
                               MIN_STEERING_ENCODER_COUNT,
                               MAX_STEERING_ENCODER_COUNT);
  Wire.requestFrom(DEVICE_NUMBER,BYTES);
  receiveEvent();
  sent.y = steering_encoder_count;
  pos.publish(&sent);
  nh.spinOnce();
  delay(MAIN_LOOP_DELAY);
}

///////////////////////////////////////////////////I2c recieve////////////////////////////////////////

void receiveEvent() {
  while (1 < Wire.available()) {
    char  x = Wire.read();
  }
  int  drive_data = Wire.read();
  sent.x = drive_data;
}
///////////////////////////////////////////////////steer encoder read channel A////////////////////////////////////////

void readChannelA() {

  if (digitalRead(INT1_PIN) == LOW) {
    tick_count++;
  } else {
    tick_count--;
  }
}
///////////////////////////////////////////////////Steer encoder read channel B////////////////////////////////////////

void readChannelB() {

  if (digitalRead(INT0_PIN) == LOW) {
    tick_count--;
  } else {
    tick_count++;
  }
}
