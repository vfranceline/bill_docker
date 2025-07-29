#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>

#include <Wire.h>

// ========== PINOS ==========
// FL: Frente Esquerda
constexpr int FL_A = 32;
constexpr int FL_B = 35;
constexpr int FL_PWM = 13;
constexpr int FL_FWD = 26;
constexpr int FL_REV = 25;
// FR: Frente Direita
constexpr int FR_A = 4;
constexpr int FR_B = 5;
constexpr int FR_PWM = 23;
constexpr int FR_FWD = 19;
constexpr int FR_REV = 18;
// RL: Traseira Esquerda
constexpr int RL_A = 21;
constexpr int RL_B = 22;
constexpr int RL_PWM = 15;
constexpr int RL_FWD = 17;
constexpr int RL_REV = 16;
// RR: Traseira Direita
constexpr int RR_A = 33;
constexpr int RR_B = 34;
constexpr int RR_PWM = 12;
constexpr int RR_FWD = 27;
constexpr int RR_REV = 14;

// ========== CINEMÁTICA MECANUM ==========
constexpr float wheel_radius = 0.07f;   // m
constexpr float lx = 0.2f;              // m
constexpr float ly = 0.175f;            // m
constexpr float L = lx + ly;            // m
constexpr int ticks_per_rev = 360;      // ajuste ao seu encoder

// ========== VARIÁVEIS DE ODOMETRIA ==========
volatile long cntFL = 0, cntFR = 0, cntRL = 0, cntRR = 0;
long lastFL = 0, lastFR = 0, lastRL = 0, lastRR = 0;

float x_pos = 0.0f, y_pos = 0.0f, theta = 0.0f;
unsigned long prevMillis = 0;

// ========== VARIÁVEIS DE VELOCIDADE DESEJADA ==========
float cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_omega = 0.0f;

// ========== MICRO-ROS ==========
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
rcl_subscription_t cmdvel_sub;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__Twist cmdvel_msg;

void fl_isr() {
  bool b = digitalRead(FL_B);
  cntFL += (b ? 1 : -1);
}
void fr_isr() {
  bool b = digitalRead(FR_B);
  cntFR += (b ? 1 : -1);
}
void rl_isr() {
  bool b = digitalRead(RL_B);
  cntRL += (b ? 1 : -1);
}
void rr_isr() {
  bool b = digitalRead(RR_B);
  cntRR += (b ? 1 : -1);
}

void cmdvel_callback(const void* msgin) {
  const auto *tw = (const geometry_msgs__msg__Twist*)msgin;
  cmd_vx = tw->linear.x;
  cmd_vy = tw->linear.y;
  cmd_omega = tw->angular.z;
  // Aqui você pode converter (vx, vy, omega) em comandos PWM para as 4 rodas
}

void setup() {
  // --- Configurações físicas ---
  pinMode(FL_A, INPUT_PULLUP); pinMode(FL_B, INPUT_PULLUP);
  pinMode(FR_A, INPUT_PULLUP); pinMode(FR_B, INPUT_PULLUP);
  pinMode(RL_A, INPUT_PULLUP); pinMode(RL_B, INPUT_PULLUP);
  pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

  pinMode(FL_PWM, OUTPUT); pinMode(FL_FWD, OUTPUT); pinMode(FL_REV, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_FWD, OUTPUT); pinMode(FR_REV, OUTPUT);
  pinMode(RL_PWM, OUTPUT); pinMode(RL_FWD, OUTPUT); pinMode(RL_REV, OUTPUT);
  pinMode(RR_PWM, OUTPUT); pinMode(RR_FWD, OUTPUT); pinMode(RR_REV, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(FL_A), fl_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_A), fr_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(RL_A), rl_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(RR_A), rr_isr, RISING);

  // --- Inicializa micro-ROS ---
  set_microros_transports();  // configure sua transport
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "esp32_mecanum", "", &support);

  // Publisher /odom
  rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom");

  // Publisher /tf
  rclc_publisher_init_default(
    &tf_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf");

  // Subscriber cmd_vel
  rclc_subscription_init_default(
    &cmdvel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  // Prepara mensagens
  nav_msgs__msg__Odometry__init(&odom_msg);
  tf2_msgs__msg__TFMessage__init(&tf_msg);

  // Executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &cmdvel_sub,
    &cmdvel_msg, &cmdvel_callback,
    ON_NEW_DATA);

  prevMillis = millis();
}

void loop() {
  // 1) Roda micro-ROS
  rclc_executor_spin_some(&RCLC_EXECUTOR_DEFAULT, RCL_MS_TO_NS(10));

  // 2) Tempo e dt
  unsigned long now = millis();
  float dt = (now - prevMillis) / 1000.0f;
  if (dt < 0.02f) return;  // ~50 Hz
  prevMillis = now;

  // 3) Obter deltas de encoder
  noInterrupts();
  long cfl = cntFL, cfr = cntFR, crl = cntRL, crr = cntRR;
  interrupts();
  long dfl = cfl - lastFL;
  long dfr = cfr - lastFR;
  long drl = crl - lastRL;
  long drr = crr - lastRR;
  lastFL = cfl; lastFR = cfr; lastRL = crl; lastRR = crr;

  // 4) Velocidades angulares (rad/s)
  float w_fl = 2*M_PI * dfl / (ticks_per_rev * dt);
  float w_fr = 2*M_PI * dfr / (ticks_per_rev * dt);
  float w_rl = 2*M_PI * drl / (ticks_per_rev * dt);
  float w_rr = 2*M_PI * drr / (ticks_per_rev * dt);

  // 5) Cinemática mecanum
  float vx = wheel_radius * (w_fl + w_fr + w_rl + w_rr) / 4.0f;
  float vy = wheel_radius * (-w_fl + w_fr + w_rl - w_rr) / 4.0f;
  float omega = wheel_radius * (-w_fl + w_fr - w_rl + w_rr) / (4.0f * L);

  // 6) Integração da pose
  float dx = (vx*cos(theta) - vy*sin(theta)) * dt;
  float dy = (vx*sin(theta) + vy*cos(theta)) * dt;
  float dth = omega * dt;
  x_pos += dx;  y_pos += dy;  theta += dth;

  // 7) Monta e publica Odometry
  odom_msg.header.stamp.sec = now/1000;
  odom_msg.header.stamp.nanosec = (now%1000)*1000000;
  odom_msg.header.frame_id.data = "odom";
  odom_msg.child_frame_id.data = "base_link";

  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0.0f;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;

  rcl_publish(&odom_pub, &odom_msg, NULL);

  // 8) Monta e publica TF (simples, apenas odom→base_link)
  geometry_msgs__msg__TransformStamped tfs;
  tfs.header = odom_msg.header;
  tfs.child_frame_id.data = "base_link";
  tfs.transform.translation.x = x_pos;
  tfs.transform.translation.y = y_pos;
  tfs.transform.translation.z = 0.0f;
  // quaternion de yaw=theta
  tfs.transform.rotation.x = 0.0f;
  tfs.transform.rotation.y = 0.0f;
  tfs.transform.rotation.z = sin(theta/2.0f);
  tfs.transform.rotation.w = cos(theta/2.0f);

  tf_msg.transforms.data = &tfs;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;
  rcl_publish(&tf_pub, &tf_msg, NULL);

    // 9) Controle dos motores a partir de cmd_vx, cmd_vy, cmd_omega
  // ------------------------------------------------------------

  // a) Cálculo da velocidade angular desejada de cada roda (rad/s)
  //    Fórmulas inversas da cinemática mecanum:
  //
  //    ω_fl = ( vx - vy - L*ωz ) / r
  //    ω_fr = ( vx + vy + L*ωz ) / r
  //    ω_rl = ( vx + vy - L*ωz ) / r
  //    ω_rr = ( vx - vy + L*ωz ) / r
  //
  float w_fl_cmd = ( cmd_vx - cmd_vy - L*cmd_omega ) / wheel_radius;
  float w_fr_cmd = ( cmd_vx + cmd_vy + L*cmd_omega ) / wheel_radius;
  float w_rl_cmd = ( cmd_vx + cmd_vy - L*cmd_omega ) / wheel_radius;
  float w_rr_cmd = ( cmd_vx - cmd_vy + L*cmd_omega ) / wheel_radius;

  // b) Conversão de ω (rad/s) para PWM (0–255) via controlador proporcional simples
  //    Ajuste Kp para calibrar resposta
  const float Kp = 50.0f;  // ganho proporcional (tune conforme seu motor/driver)

  int pwm_fl = constrain(int( Kp * w_fl_cmd ), -255, 255);
  int pwm_fr = constrain(int( Kp * w_fr_cmd ), -255, 255);
  int pwm_rl = constrain(int( Kp * w_rl_cmd ), -255, 255);
  int pwm_rr = constrain(int( Kp * w_rr_cmd ), -255, 255);

  // c) Escrita nos pinos de direção e PWM
  auto applyMotor = [&](int pwm, int pin_pwm, int pin_fwd, int pin_rev){
    if(pwm >= 0) {
      digitalWrite(pin_fwd, HIGH);
      digitalWrite(pin_rev, LOW);
      analogWrite(pin_pwm, pwm);
    } else {
      digitalWrite(pin_fwd, LOW);
      digitalWrite(pin_rev, HIGH);
      analogWrite(pin_pwm, -pwm);
    }
  };

  applyMotor(pwm_fl, FL_PWM, FL_FWD, FL_REV);
  applyMotor(pwm_fr, FR_PWM, FR_FWD, FR_REV);
  applyMotor(pwm_rl, RL_PWM, RL_FWD, RL_REV);
  applyMotor(pwm_rr, RR_PWM, RR_FWD, RR_REV);

}
