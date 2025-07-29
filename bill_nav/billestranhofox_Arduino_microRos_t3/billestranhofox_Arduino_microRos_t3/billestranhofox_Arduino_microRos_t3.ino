
/*
  Connect ESP32 to ROS2 – microROS (humble) com Arduino
  Referência
  Differential drive robot using ROS2, micro-ROS and ESP32
  https://www.hackster.io/amal-shaji/differential-drive-robot-using-ros2-and-esp32-aae289

  Adaptação: JS, 2024
*/
/*Motor controller using micro_ros serial set_microros_transports*/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <odometry.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>


//pin declaration
//roda_Dianteira_
//Left wheel
int8_t roda_Dianteira_L_FORW = 26;
int8_t roda_Dianteira_L_BACK = 25;
int8_t roda_Dianteira_L_enablePin = 13;
int8_t roda_Dianteira_L_encoderPin1 = 32;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t roda_Dianteira_L_encoderPin2 = 35;

//Right wheel
int8_t roda_Dianteira_R_FORW = 19;
int8_t roda_Dianteira_R_BACK = 18;
int8_t roda_Dianteira_R_enablePin = 23;
int8_t roda_Dianteira_R_encoderPin1  = 4;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t roda_Dianteira_R_encoderPin2 = 5;

//roda_Traseira
//Left wheel
int8_t roda_Traseira_L_FORW = 17;
int8_t roda_Traseira_L_BACK = 16;
int8_t roda_Traseira_L_enablePin = 15;
int8_t roda_Traseira_L_encoderPin1 = 21;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t roda_Traseira_L_encoderPin2 = 22;

//Right wheel
int8_t roda_Traseira_R_FORW = 12;
int8_t roda_Traseira_R_BACK = 27;
int8_t roda_Traseira_R_enablePin = 14;
int8_t roda_Traseira_R_encoderPin1  = 34;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t roda_Traseira_R_encoderPin2 = 33;


//parameters of the robot
//float wheels_y_distance_ = 0.1;
float wheels_y_distance_ = 0.35; //jurema medido em 10/07/2024
//float wheel_radius = 0.02;
float wheel_radius = 0.048; //jurema medido em 10/07/2024
float wheel_circumference_ = 2 * 3.14 * wheel_radius;
//Para teste
//jurema medido em 03/09/2024
#define axis_lenght 0.22
float half_axis_separation = wheels_y_distance_ / 2;
float half_axis_length = axis_lenght / 2;

//encoder value per revolution of left wheel and right wheel
//roda_Dianteira
int roda_Dianteira_tickPerRevolution_LW = 663; //teórico jurema //Original 1055;
int roda_Dianteira_tickPerRevolution_RW = 663; //te[orico jurema //Original 1055;
//roda_Traseira
int roda_Traseira_tickPerRevolution_LW = 663; //teórico jurema //Original 1055;
int roda_Traseira_tickPerRevolution_RW = 663; //teórico jurema //Original 1055;

//int threshold = 0;

//threshold é um valor que foi encontrado ao implementar este projeto.
//o motor não se move quando o valor pwm está entre 0 e 165.
//Quando o valor pwm é maior que 165, o motor começa a se mover. Testado com 150
int threshold = 180; //200; //200; //165; //120; //0; //180; //100; //50; //150; //default = 0;

/*
  //roda_dianteira
  //pid constants of left wheel
  float roda_Dianteira_kp_l = 1.8;
  float roda_Dianteira_ki_l = 5;thres
  float roda_Dianteira_kd_l = 0.1;
  //pid constants of right wheel
  float roda_Dianteira_kp_r = 2.25;
  float roda_Dianteira_ki_r = 5;
  float roda_Dianteira_kd_r = 0.1;

  //roda_Traseira
  //pid constants of left wheel
  float roda_Traseira_kp_l = 1.8;
  float roda_Traseira_ki_l = 5;
  float roda_Traseira_kd_l = 0.1;
  //pid constants of right wheel
  float roda_Traseira_kp_r = 2.25;
  float roda_Traseira_ki_r = 5;
  float roda_Traseira_kd_r = 0.1;
*/

//roda_dianteira
//pid constants of left wheel
float roda_Dianteira_kp_l = 1.0;
float roda_Dianteira_ki_l = 5;
float roda_Dianteira_kd_l = 0.1;
//pid constants of right wheel
float roda_Dianteira_kp_r = 1.5;
float roda_Dianteira_ki_r = 5;
float roda_Dianteira_kd_r = 0.1;

//roda_Traseira
//pid constants of left wheel
float roda_Traseira_kp_l = 1.0;
float roda_Traseira_ki_l = 5;
float roda_Traseira_kd_l = 0.1;
//pid constants of right wheel
float roda_Traseira_kp_r = 1.5;
float roda_Traseira_ki_r = 5;
float roda_Traseira_kd_r = 0.1;


//pwm parameters setup
const int freq = 30000;
const int roda_Dianteira_pwmChannelL = 0;
const int roda_Dianteira_pwmChannelR = 1;
const int roda_Traseira_pwmChannelL = 2;
const int roda_Traseira_pwmChannelR = 3;

const int resolution = 8;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t odom_publisher;
//std_msgs__msg__Int32 encodervalue_l;
//std_msgs__msg__Int32 encodervalue_r;
//roda_Dianteira
std_msgs__msg__Int32 roda_Dianteira_encodervalue_l;  //JS
std_msgs__msg__Int32 roda_Dianteira_encodervalue_r;  //JS
//roda_Traseira
std_msgs__msg__Int32 roda_Traseira_encodervalue_l;  //JS
std_msgs__msg__Int32 roda_Traseira_encodervalue_r;  //JS


nav_msgs__msg__Odometry odom_msg;
rcl_timer_t timer;
rcl_timer_t ControlTimer;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;

//creating a class for motor control
class MotorController {
  public:
    int8_t Forward;
    int8_t Backward;
    int8_t Enable;
    int8_t EncoderPinA;
    int8_t EncoderPinB;
    std_msgs__msg__Int32 EncoderCount;
    volatile long CurrentPosition;
    volatile long PreviousPosition;
    volatile long CurrentTime;
    volatile long PreviousTime;
    volatile long CurrentTimeforError;
    volatile long PreviousTimeForError;
    float rpmFilt;
    float eintegral;
    float ederivative;
    float rpmPrev;
    float kp;
    float ki;
    float kd;
    float error;
    float previousError = 0;
    int tick;

    MotorController(int8_t ForwardPin, int8_t BackwardPin, int8_t EnablePin, int8_t EncoderA, int8_t EncoderB, int tickPerRevolution) {
      this->Forward = ForwardPin;
      this->Backward = BackwardPin;
      this->Enable = EnablePin;
      this->EncoderPinA = EncoderA;
      this->EncoderPinB = EncoderB;
      this->tick = tickPerRevolution;
      pinMode(Forward, OUTPUT);
      pinMode(Backward, OUTPUT);
      pinMode(EnablePin, OUTPUT);
      pinMode(EncoderPinA, INPUT);
      pinMode(EncoderPinB, INPUT);
    }

    //initializing the parameters of PID controller
    void initPID(float proportionalGain, float integralGain, float derivativeGain) {
      kp = proportionalGain;
      ki = integralGain;
      kd = derivativeGain;
    }

    //function return rpm of the motor using the encoder tick values
    float getRpm() {
      CurrentPosition = EncoderCount.data;
      CurrentTime = millis();
      float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3;
      float velocity = ((float)CurrentPosition - PreviousPosition) / delta1;
      float rpm = (velocity / tick) * 60;
      rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
      float rpmPrev = rpm;
      PreviousPosition = CurrentPosition;
      PreviousTime = CurrentTime;
      // Serial.println(rpmFilt);
      return rpmFilt;
    }

    //pid controller
    float pid(float setpoint, float feedback) {
      CurrentTimeforError = millis();
      float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
      error = setpoint - feedback;
      eintegral = eintegral + (error * delta2);
      ederivative = (error - previousError) / delta2;
      float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

      previousError = error;
      PreviousTimeForError = CurrentTimeforError;
      return control_signal;
    }
    //move the robot wheels based the control signal generated by the pid controller
    void moveBase(float ActuatingSignal, int threshold, int pwmChannel) {
      if (ActuatingSignal > 0) {
        digitalWrite(Forward, HIGH);
        digitalWrite(Backward, LOW);
      } else {
        digitalWrite(Forward, LOW);
        digitalWrite(Backward, HIGH);
      }
      int pwm = threshold + (int)fabs(ActuatingSignal);
      if (pwm > 255)
        pwm = 255;
      //pwm = 200; //JS
      ledcWrite(pwmChannel, pwm);
    }
    void stop() {
      digitalWrite(Forward, LOW);
      digitalWrite(Backward, LOW);
    }

    // void plot(float Value1, float Value2){
    //     Serial.print("Value1:");
    //     Serial.print(Value1);
    //     Serial.print(",");
    //     Serial.print("value2:");
    //     Serial.println(Value2);
    // }
};

//creating objects for right wheel and left wheel
//roda_Dianteira
MotorController roda_Dianteira_leftWheel(roda_Dianteira_L_FORW, roda_Dianteira_L_BACK, roda_Dianteira_L_enablePin, roda_Dianteira_L_encoderPin1, roda_Dianteira_L_encoderPin2, roda_Dianteira_tickPerRevolution_LW);
MotorController roda_Dianteira_rightWheel(roda_Dianteira_R_FORW, roda_Dianteira_R_BACK, roda_Dianteira_R_enablePin, roda_Dianteira_R_encoderPin1 , roda_Dianteira_R_encoderPin2, roda_Dianteira_tickPerRevolution_RW);

//roda_Traseira
MotorController roda_Traseira_leftWheel(roda_Traseira_L_FORW, roda_Traseira_L_BACK, roda_Traseira_L_enablePin, roda_Traseira_L_encoderPin1, roda_Traseira_L_encoderPin2, roda_Traseira_tickPerRevolution_LW);
MotorController roda_Traseira_rightWheel(roda_Traseira_R_FORW, roda_Traseira_R_BACK, roda_Traseira_R_enablePin, roda_Traseira_R_encoderPin1 , roda_Traseira_R_encoderPin2, roda_Traseira_tickPerRevolution_RW);

#define LED_PIN 2

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//subscription callback function

void setup() {

  // Start serial communication with a baud rate of 115200
  //Serial.begin(115200);



  //initializing the pid constants
  //roda_Dianteira
  roda_Dianteira_leftWheel.initPID(roda_Dianteira_kp_l, roda_Dianteira_ki_l, roda_Dianteira_kd_l);
  roda_Dianteira_rightWheel.initPID(roda_Dianteira_kp_r, roda_Dianteira_ki_r, roda_Dianteira_kd_r);
  //roda_Traseira
  roda_Traseira_leftWheel.initPID(roda_Traseira_kp_l, roda_Traseira_ki_l, roda_Traseira_kd_l);
  roda_Traseira_rightWheel.initPID(roda_Traseira_kp_r, roda_Traseira_ki_r, roda_Traseira_kd_r);


  //initializing interrupt functions for counting the encoder tick values
  //roda_Dianteira
  attachInterrupt(digitalPinToInterrupt(roda_Dianteira_leftWheel.EncoderPinB), roda_Dianteira_updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(roda_Dianteira_rightWheel.EncoderPinA), roda_Dianteira_updateEncoderR, RISING);
  //roda_Traseira
  attachInterrupt(digitalPinToInterrupt(roda_Traseira_leftWheel.EncoderPinB), roda_Traseira_updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(roda_Traseira_rightWheel.EncoderPinA), roda_Traseira_updateEncoderR, RISING);

  //initializing pwm signal parameters
  //roda_Dianteira
  ledcSetup(roda_Dianteira_pwmChannelL, freq, resolution);
  ledcAttachPin(roda_Dianteira_leftWheel.Enable, roda_Dianteira_pwmChannelL);
  ledcSetup(roda_Dianteira_pwmChannelR, freq, resolution);
  ledcAttachPin(roda_Dianteira_rightWheel.Enable, roda_Dianteira_pwmChannelR);
  //roda_Traseira
  ledcSetup(roda_Traseira_pwmChannelL, freq, resolution);
  ledcAttachPin(roda_Traseira_leftWheel.Enable, roda_Traseira_pwmChannelL);
  ledcSetup(roda_Traseira_pwmChannelR, freq, resolution);
  ledcAttachPin(roda_Traseira_rightWheel.Enable, roda_Traseira_pwmChannelR);

  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create subscriber for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"));
  //create a odometry publisher
  RCCHECK(rclc_publisher_init_default(
            &odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "odom/unfiltered"));

  //timer function for controlling the motor base. At every samplingT time
  //MotorControll_callback function is called
  //Here I had set SamplingT=10 Which means at every 10 milliseconds MotorControll_callback function is called
  //const unsigned int samplingT = 20;
  const unsigned int samplingT = 10; //JS


  RCCHECK(rclc_timer_init_default(
            &ControlTimer,
            &support,
            RCL_MS_TO_NS(samplingT),
            MotorControll_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));
}

void loop() {
  // put your main code here, to run repeatedly:

  //delay(100);
  delay(10); //JS
  //RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  //A macro RCL_MS_TO_NS no ROS2 é usada para converter milissegundos em nanosegundos.
  //Isso é útil em várias situações onde você precisa definir períodos de tempo com alta precisão, como ao configurar timers.
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); //JS

}

void subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
}

//function which controlles the motor
void MotorControll_callback(rcl_timer_t* timer, int64_t last_call_time) {
  float linearVelocity_x;
  float linearVelocity_y; //JS - incluida
  float angularVelocity_z;

  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity_x = msg.linear.x;
  angularVelocity_z = msg.angular.z;
  linearVelocity_y = msg.linear.y; //JS  incluida


  //linear and angular velocities are converted to roda_Dianteira_leftWheel and roda_Dianteira_rightWheel velocities
  /*
    //Original
    float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
    float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;
  */
  /*
    //Original adaptada
    //roda Dianteira
    float roda_Dianteira_vL = (linearVelocity_x - (angularVelocity_z * 1 / 2)) * 20; //JS
    float roda_Dianteira_vR = (linearVelocity_x + (angularVelocity_z * 1 / 2)) * 20; // JS
    //roda Traseira
    float roda_Traseira_vL = (linearVelocity_x - (angularVelocity_z * 1 / 2)) * 20; //JS
    float roda_Traseira_vR = (linearVelocity_x + (angularVelocity_z * 1 / 2)) * 20; // JS
  */
  /*
        //Teste 2
      //roda Dianteira
      float roda_Dianteira_vL = (linearVelocity_x - (angularVelocity_z * 1 / 2)) * 20; //JS
      float roda_Dianteira_vR = (linearVelocity_x + (angularVelocity_z * 1 / 2)) * 20; // JS
      //roda Traseira
      float roda_Traseira_vL = (linearVelocity_x - (angularVelocity_z * 1 / 2)) * 20; //JS
      float roda_Traseira_vR = (linearVelocity_x + (angularVelocity_z * 1 / 2)) * 20; // JS
  */

  // //original
  // //A ser utilizada sem PID
  // float roda_Dianteira_vL = (linearVelocity_x - angularVelocity_z - linearVelocity_y) * 10; //JS
  // float roda_Dianteira_vR = (linearVelocity_x + angularVelocity_z + linearVelocity_y) * 10; // JS
  // //roda Traseira
  // float roda_Traseira_vL =  (linearVelocity_x - angularVelocity_z + linearVelocity_y) * 10; //JS
  // float roda_Traseira_vR =  (linearVelocity_x + angularVelocity_z - linearVelocity_y) * 10; // JS

  // //teste 1 - vf - ainda com movimento no sentido inverso
  // float roda_Dianteira_vL = (linearVelocity_x - angularVelocity_z - linearVelocity_y) * 10; //JS
  // float roda_Dianteira_vR = (linearVelocity_x + angularVelocity_z + linearVelocity_y) * 10; // JS
  // //roda Traseira
  // float roda_Traseira_vL =  (linearVelocity_x + angularVelocity_z - linearVelocity_y) * 10; //JS
  // float roda_Traseira_vR =  (linearVelocity_x - angularVelocity_z + linearVelocity_y) * 10; // JS

  //teste 2 - vf
  float roda_Dianteira_vL = (linearVelocity_x + angularVelocity_z + linearVelocity_y) * 10; //JS
  float roda_Dianteira_vR = (linearVelocity_x - angularVelocity_z - linearVelocity_y) * 10; // JS
  //roda Traseira
  float roda_Traseira_vL =  (linearVelocity_x + angularVelocity_z - linearVelocity_y) * 10; //JS
  float roda_Traseira_vR =  (linearVelocity_x - angularVelocity_z + linearVelocity_y) * 10; // JS

  /*
    //Funciona +-
    //JS - Testando, equação base foi a utilizada em fox_t4
    //float front_left_speed = (1 / WHEEL_RADIUS) * (linear_x - linear_y - (half_axis_separation+half_axis_length) * angular_z);
    //roda Dianteira
    //linear and angular velocities are converted to roda Dianteira leftWheel (roda_Dianteira_vL) and roda Dianteira rightWheel(roda_Dianteira_vR) velocities
    float roda_Dianteira_vL = ((1 / wheel_radius) * (linearVelocity_x - linearVelocity_y  - (half_axis_separation + half_axis_length) * angularVelocity_z)) * 10;
    float roda_Dianteira_vR = ((1 / wheel_radius) * (linearVelocity_x + linearVelocity_y  + (half_axis_separation + half_axis_length) * angularVelocity_z)) * 10;
    //roda Traseira
    //linear and angular velocities are converted to roda Traseira leftWheel (roda_Traseira_vL) and roda Traseira rightWheel(roda_Traseira_vR) velocities
    float roda_Traseira_vL = ((1 / wheel_radius) * (linearVelocity_x - linearVelocity_y  - (half_axis_separation + half_axis_length) * angularVelocity_z)) * 10;
    float roda_Traseira_vR = ((1 / wheel_radius) * (linearVelocity_x + linearVelocity_y  + (half_axis_separation + half_axis_length) * angularVelocity_z)) * 10;
    //JS - Testando, equação base foi a utilizada em fox_t4
  */

  //current wheel rpm is calculated
  /*
    float currentRpmL = roda_Dianteira_leftWheel.getRpm();
    float currentRpmR = roda_Dianteira_rightWheel.getRpm();
  */
  //current wheel rpm is calculated //JS
  //roda Dianteira
  float roda_Dianteira_currentRpmL = roda_Dianteira_leftWheel.getRpm();
  float roda_Dianteira_currentRpmR = roda_Dianteira_rightWheel.getRpm();
  //roda Traseira
  float roda_Traseira_currentRpmL = roda_Traseira_leftWheel.getRpm();
  float roda_Traseira_currentRpmR = roda_Traseira_rightWheel.getRpm();

  /*
    //pid controlled is used for generating the pwm signal
    //roda dianteira
    float roda_Dianteira_actuating_signal_LW = roda_Dianteira_leftWheel.pid(roda_Dianteira_vL, currentRpmL);
    float roda_Dianteira_actuating_signal_RW = roda_Dianteira_rightWheel.pid(roda_Dianteira_vR, currentRpmR);
    //roda_Traseira
    float roda_Traseira_actuating_signal_LW = roda_Traseira_leftWheel.pid(roda_Traseira_vL, currentRpmL);
    float roda_Traseira_actuating_signal_RW = roda_Traseira_rightWheel.pid(roda_Traseira_vR, currentRpmR);
  */

  /*
    //pid controlled is used for generating the pwm signal //JS
    //roda dianteira
    float roda_Dianteira_actuating_signal_LW = roda_Dianteira_leftWheel.pid(roda_Dianteira_vL, roda_Dianteira_currentRpmL);
    float roda_Dianteira_actuating_signal_RW = roda_Dianteira_rightWheel.pid(roda_Dianteira_vR, roda_Dianteira_currentRpmR);
    //roda_Traseira
    float roda_Traseira_actuating_signal_LW = roda_Traseira_leftWheel.pid(roda_Traseira_vL, roda_Traseira_currentRpmL);
    float roda_Traseira_actuating_signal_RW = roda_Traseira_rightWheel.pid(roda_Traseira_vR, roda_Traseira_currentRpmR);
  */


  //generating the pwm signal sem PID //JS
  //roda dianteira
  float roda_Dianteira_actuating_signal_LW = roda_Dianteira_vL; //JS
  float roda_Dianteira_actuating_signal_RW = roda_Dianteira_vR; //JS
  //roda_Traseira
  float roda_Traseira_actuating_signal_LW =  roda_Traseira_vL;  //JS
  float roda_Traseira_actuating_signal_RW =  roda_Traseira_vR;  //JS


  if (roda_Dianteira_vL == 0 && roda_Dianteira_vR == 0 && roda_Traseira_vL == 0 && roda_Traseira_vR == 0) {
    //if (linearVelocity_x == 0 && angularVelocity_z == 0 && linearVelocity_y == 0) {
    //roda_Dianteira
    roda_Dianteira_leftWheel.stop();
    roda_Dianteira_rightWheel.stop();
    roda_Dianteira_actuating_signal_LW = 0;
    roda_Dianteira_actuating_signal_RW = 0;
    //roda_Traseira
    roda_Traseira_leftWheel.stop();
    roda_Traseira_rightWheel.stop();
    roda_Traseira_actuating_signal_LW = 0;
    roda_Traseira_actuating_signal_RW = 0;
  }


  
  
  /* Teste 20/01/2025
  else {
    //else  if ((linearVelocity_x != 0 || angularVelocity_z != 0) && linearVelocity_y == 0) {
    //roda_Dianteira
    roda_Dianteira_rightWheel.moveBase(roda_Dianteira_actuating_signal_RW, threshold, roda_Dianteira_pwmChannelR);
    roda_Dianteira_leftWheel.moveBase(roda_Dianteira_actuating_signal_LW, threshold, roda_Dianteira_pwmChannelL);
    //roda_Traseira
    roda_Traseira_rightWheel.moveBase(roda_Traseira_actuating_signal_RW, threshold, roda_Traseira_pwmChannelR);
    roda_Traseira_leftWheel.moveBase(roda_Traseira_actuating_signal_LW, threshold, roda_Traseira_pwmChannelL);
  }

*/
else {
    //else  if ((linearVelocity_x != 0 || angularVelocity_z != 0) && linearVelocity_y == 0) {
    //roda_Dianteira
    roda_Dianteira_rightWheel.moveBase(roda_Dianteira_actuating_signal_LW, threshold, roda_Dianteira_pwmChannelR);
    roda_Dianteira_leftWheel.moveBase(roda_Dianteira_actuating_signal_RW, threshold, roda_Dianteira_pwmChannelL);
    //roda_Traseira
    roda_Traseira_rightWheel.moveBase(roda_Traseira_actuating_signal_LW, threshold, roda_Traseira_pwmChannelR);
    roda_Traseira_leftWheel.moveBase(roda_Traseira_actuating_signal_RW, threshold, roda_Traseira_pwmChannelL);
  }


  /*
    else  if ((linearVelocity_x != 0 || angularVelocity_z != 0) && linearVelocity_y != 0) {
    //roda_Dianteira
    roda_Dianteira_rightWheel.moveBase(roda_Dianteira_actuating_signal_RW, threshold, roda_Dianteira_pwmChannelR);
    //roda_Dianteira_leftWheel.moveBase(roda_Dianteira_actuating_signal_LW, threshold, roda_Dianteira_pwmChannelL);
    roda_Dianteira_leftWheel.moveBase(roda_Dianteira_actuating_signal_RW, threshold, roda_Dianteira_pwmChannelL);
    //roda_Traseira
    roda_Traseira_rightWheel.moveBase(roda_Traseira_actuating_signal_RW, threshold, roda_Traseira_pwmChannelR);
    //roda_Traseira_leftWheel.moveBase(roda_Traseira_actuating_signal_LW, threshold, roda_Traseira_pwmChannelL);
    roda_Traseira_leftWheel.moveBase(roda_Traseira_actuating_signal_RW, threshold, roda_Traseira_pwmChannelL);
    }
  */

  //odometry
  //float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0; // RPM
  float average_rps_x = ((float)(roda_Dianteira_currentRpmL + roda_Dianteira_currentRpmR + roda_Traseira_currentRpmL + roda_Traseira_currentRpmR ) / 4) / 120.0;  // RPM //JS
  float linear_x = average_rps_x * wheel_circumference_;                  // m/s
  //float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
  float average_rps_a = ((float)(((-roda_Dianteira_currentRpmL + roda_Dianteira_currentRpmR) + ( -  roda_Traseira_currentRpmL + roda_Traseira_currentRpmR)) ) / 4) / 120.0;  // RPM JS
  float angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0);  //  rad/s
  float linear_y = 0;
  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(
    vel_dt,
    linear_x,
    linear_y,
    angular_z);
  publishData();
}

//roda_Dianteira
//interrupt function for left wheel encoder.
void roda_Dianteira_updateEncoderL() {
  if (digitalRead(roda_Dianteira_leftWheel.EncoderPinB) > digitalRead(roda_Dianteira_leftWheel.EncoderPinA))
    roda_Dianteira_leftWheel.EncoderCount.data++;
  else
    roda_Dianteira_leftWheel.EncoderCount.data--;
  roda_Dianteira_encodervalue_l = roda_Dianteira_leftWheel.EncoderCount;
}

//interrupt function for right wheel encoder
void roda_Dianteira_updateEncoderR() {
  if (digitalRead(roda_Dianteira_rightWheel.EncoderPinA) > digitalRead(roda_Dianteira_rightWheel.EncoderPinB))
    roda_Dianteira_rightWheel.EncoderCount.data++;
  else
    roda_Dianteira_rightWheel.EncoderCount.data--;
  roda_Dianteira_encodervalue_r = roda_Dianteira_rightWheel.EncoderCount;
}


//roda_Traseira
//interrupt function for left wheel encoder.
void roda_Traseira_updateEncoderL() {
  if (digitalRead(roda_Traseira_leftWheel.EncoderPinB) > digitalRead(roda_Traseira_leftWheel.EncoderPinA))
    roda_Traseira_leftWheel.EncoderCount.data++;
  else
    roda_Traseira_leftWheel.EncoderCount.data--;
  roda_Traseira_encodervalue_l = roda_Traseira_leftWheel.EncoderCount;
}

//interrupt function for right wheel encoder
void roda_Traseira_updateEncoderR() {
  if (digitalRead(roda_Traseira_rightWheel.EncoderPinA) > digitalRead(roda_Traseira_rightWheel.EncoderPinB))
    roda_Traseira_rightWheel.EncoderCount.data++;
  else
    roda_Traseira_rightWheel.EncoderCount.data--;
  roda_Traseira_encodervalue_r = roda_Traseira_rightWheel.EncoderCount;
}


//function which publishes wheel odometry.
void publishData() {
  odom_msg = odometry.getData();
  ;

  struct timespec time_stamp = getTime();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}
void syncTime() {
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime() {
  struct timespec tp = { 0 };
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}
