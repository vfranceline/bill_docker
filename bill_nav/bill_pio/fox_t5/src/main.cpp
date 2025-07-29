#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include "motor_pins.h"
#include "encoder_pins.h"
#include "motor_control.h"
#include "MecanumControl.h"

// --- OBJETOS E CONSTANTES ---
#define NUM_ENCODERS 4
#define TICKS_PER_REVOLUTION 663 // Valor corrigido para o seu encoder

// Objeto para controle da base Mecanum
MecanumControl mecanumRobot(
    MOTOR1_PWM, MOTOR1_FWD, MOTOR1_REV,
    MOTOR2_PWM, MOTOR2_FWD, MOTOR2_REV,
    MOTOR3_PWM, MOTOR3_FWD, MOTOR3_REV,
    MOTOR4_PWM, MOTOR4_FWD, MOTOR4_REV
);

// Pinos dos Encoders
int encoderPinsA[] = {ENCODER_A_1, ENCODER_A_2, ENCODER_A_3, ENCODER_A_4};
int encoderPinsB[] = {ENCODER_B_1, ENCODER_B_2, ENCODER_B_3, ENCODER_B_4};

// Variáveis dos Encoders
volatile int encoder_counts[NUM_ENCODERS] = {0, 0, 0, 0};
volatile long last_micros[NUM_ENCODERS] = {0, 0, 0, 0};

// --- VARIÁVEIS DO MICRO-ROS ---
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Publishers (Encoder e Velocidade)
rcl_publisher_t encoder_publisher;
rcl_publisher_t vel_publisher;
std_msgs__msg__Float32MultiArray encoder_msg;
std_msgs__msg__Float32MultiArray vel_msg;

// Subscriber (cmd_vel)
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist twist_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); }}
void error_loop(){ while(1){ delay(100); } }

// --- FUNÇÕES DE INTERRUPÇÃO (ISR) PARA OS ENCODERS ---
void IRAM_ATTR readEncoder(int index) {
    digitalRead(encoderPinsB[index]) > 0 ? encoder_counts[index]++ : encoder_counts[index]--;
}

void IRAM_ATTR readEncoder1() { readEncoder(0); }
void IRAM_ATTR readEncoder2() { readEncoder(1); }
void IRAM_ATTR readEncoder3() { readEncoder(2); }
void IRAM_ATTR readEncoder4() { readEncoder(3); }


// --- CALLBACKS DO MICRO-ROS ---

// Callback para quando uma mensagem de /cmd_vel é recebida
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;  
  mecanumRobot.moveWithCmdVel(msg->linear.x, msg->linear.y, msg->angular.z);
}

// Callback do timer para ler encoders e publicar dados
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

    float motor_rpm[NUM_ENCODERS];

    for (int i = 0; i < NUM_ENCODERS; i++) {
        long current_micros = micros();
        long dt = current_micros - last_micros[i];
        last_micros[i] = current_micros;

        int counts = encoder_counts[i];
        encoder_counts[i] = 0;

        if (dt > 0) {
            motor_rpm[i] = (float(counts) / TICKS_PER_REVOLUTION) * (60.0 * 1000000.0 / dt);
        } else {
            motor_rpm[i] = 0;
        }

        encoder_msg.data.data[i] = counts;
        vel_msg.data.data[i] = motor_rpm[i];
    }
    
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    rcl_publish(&vel_publisher, &vel_msg, NULL);
}


// --- SETUP ---
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    // Configuração dos pinos dos encoders
    for (int i = 0; i < NUM_ENCODERS; i++) {
        pinMode(encoderPinsA[i], INPUT_PULLUP);
        pinMode(encoderPinsB[i], INPUT_PULLUP);
    }
    
    // Anexar as interrupções
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), readEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_3), readEncoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_4), readEncoder4, RISING);

    // Inicialização do micro-ROS
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "fox_t5", "", &support));

    // Configuração dos Publishers
    RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "encoder_counts"));
    RCCHECK(rclc_publisher_init_default(&vel_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motor_velocities_rpm"));

    // Configuração do Subscriber
    RCCHECK(rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    // Configuração do Timer
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));

    // Configuração do Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 2 = 1 subscriber + 1 timer
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Alocação de memória para as mensagens
    encoder_msg.data.size = NUM_ENCODERS;
    encoder_msg.data.capacity = NUM_ENCODERS;
    encoder_msg.data.data = (float*)malloc(encoder_msg.data.capacity * sizeof(float));

    vel_msg.data.size = NUM_ENCODERS;
    vel_msg.data.capacity = NUM_ENCODERS;
    vel_msg.data.data = (float*)malloc(vel_msg.data.capacity * sizeof(float));
}

// --- LOOP ---
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}