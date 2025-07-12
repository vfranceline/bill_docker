#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    int pwmPin, fwdPin, revPin; // Alterado de inDirPin para fwdPin e revPin
    int channel;

    // Construtor atualizado para aceitar os dois pinos de direção
    MotorControl(int pwmPin, int fwdPin, int revPin, int channel)
        : pwmPin(pwmPin), fwdPin(fwdPin), revPin(revPin), channel(channel) {
        pinMode(fwdPin, OUTPUT);
        pinMode(revPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        ledcSetup(channel, 5000, 8); // 5 kHz, 8-bit de resolução
        ledcAttachPin(pwmPin, channel);
    }

    void setSpeed(int speed) {
        ledcWrite(channel, speed);
    }

    // Move o motor para frente
    void moveForward() {
        digitalWrite(fwdPin, HIGH);
        digitalWrite(revPin, LOW);
    }

    // Move o motor para trás
    void moveBackward() {
        digitalWrite(fwdPin, LOW);
        digitalWrite(revPin, HIGH);
    }

    // Para o motor
    void stop() {
        digitalWrite(fwdPin, LOW);
        digitalWrite(revPin, LOW);
        ledcWrite(channel, 0);
    }
};

#endif // MOTOR_CONTROL_H