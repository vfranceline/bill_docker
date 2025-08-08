#ifndef MECANUM_CONTROL_H
#define MECANUM_CONTROL_H

#include "motor_control.h"

class MecanumControl {
private:
    MotorControl motor1, motor2, motor3, motor4;
    float wheel_radius = 0.075; // Raio da roda (m)
    float robot_length = 0.255; // Metade do comprimento do robô (m)
    float robot_width = 0.22;  // Metade da largura do robô (m)

    // --- PARÂMETROS DE CONTROLE DE VELOCIDADE ---

    // Velocidade máxima que as rodas podem atingir em radianos/segundo.
    // (146 RPM equivalem a ~15.3 rad/s. Usamos 16.0 como um limite seguro)
    const float MAX_RAD_PER_SEC = 16.0;

    // Fator de ajuste de velocidade. Altere este valor para tornar o robô mais rápido ou mais lento.
    // 1.0 = 100% da velocidade, 0.5 = 50%, etc.
    const float SPEED_ADJUST_FACTOR = 1;


public:
    MecanumControl(int m1_pwm, int m1_fwd, int m1_rev,
                   int m2_pwm, int m2_fwd, int m2_rev,
                   int m3_pwm, int m3_fwd, int m3_rev,
                   int m4_pwm, int m4_fwd, int m4_rev)
        : motor1(m1_pwm, m1_fwd, m1_rev, 0),
          motor2(m2_pwm, m2_fwd, m2_rev, 1),
          motor3(m3_pwm, m3_fwd, m3_rev, 2),
          motor4(m4_pwm, m4_fwd, m4_rev, 3) {}

    void moveWithCmdVel(float Vx, float Vy, float omega) {
        float L = robot_length;
        float W = robot_width;
        float r = wheel_radius;

        // Cinemática Inversa: Calcula a velocidade angular necessária para cada roda (em rad/s)
        float w1 = (Vx - Vy - (L + W) * omega) / r; // Roda FL
        float w2 = (Vx + Vy + (L + W) * omega) / r; // Roda FR
        float w3 = (Vx + Vy - (L + W) * omega) / r; // Roda RL
        float w4 = (Vx - Vy + (L + W) * omega) / r; // Roda RR

        // Envia a velocidade calculada para cada motor
        setMotorSpeed(motor1, w1);
        setMotorSpeed(motor2, w2);
        setMotorSpeed(motor3, w3);
        setMotorSpeed(motor4, w4);
    }

    void setMotorSpeed(MotorControl &motor, float rad_per_sec) {

        const int MIN_PWM = 150; // Valor mínimo de PWM (para evitar a zona morta do motor)

        // Define a direção do motor com base no sinal da velocidade
        if (rad_per_sec > 0.01) {
            motor.moveForward();
        } else if (rad_per_sec < -0.01) {
            motor.moveBackward();
        } else {
            motor.stop();
            return;
        }

        // Mapeia a velocidade angular (rad/s) para um valor PWM que começa em MIN_PWM
        // Isso garante que qualquer comando de movimento fornecerá energia suficiente para vencer a inércia.       
        long pwm_value = map(abs(rad_per_sec) * 100, 0, MAX_RAD_PER_SEC * 100, MIN_PWM, 255);

        // Garante que o valor final do PWM não seja menor que o mínimo (se o motor estiver 
        // em movimento) nem maior que o maximo permitido (255).
        int final_pwm = constrain(pwm_value, MIN_PWM, 255);

        motor.setSpeed(final_pwm);
    }
};

#endif // MECANUM_CONTROL_H