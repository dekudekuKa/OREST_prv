#include "sbus.h"
#include "Servo.h"

// Motor control pins
#define ENA 3
#define MOTORA_1 4
#define MOTORA_2 5
#define MOTORB_1 8
#define MOTORB_2 7
#define ENB 6
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define IN5 13
#define IN6 14
#define IN7 15
#define IN8 16
#define OFF 1
#define ON 0

// SBUS objects
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusTx sbus_tx(&Serial1);
bfs::SbusData data;

// Joystick variables
int joystick_left_y = 0;
int joystick_left_y_speed = 0;
int joystick_right_x = 0;
int joystick_right_x_speed = 0;

bool relay1State = false; // Стан реле 1 (вимкнене за замовчуванням)
bool relay2State = false; // Стан реле 2 (вимкнене за замовчуванням)
bool previousSwitchStateRelay1 = false; // Попередній стан для реле 1
bool previousSwitchStateRelay2 = false; // Попередній стан для реле 2

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    sbus_rx.Begin();
    sbus_tx.Begin();

    pinMode(ENA, OUTPUT);
    pinMode(MOTORA_1, OUTPUT);
    pinMode(MOTORA_2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(MOTORB_1, OUTPUT);
    pinMode(MOTORB_2, OUTPUT);

    stopMotors();

    relay_init();
}

void loop() {
    if (sbus_rx.Read()) {
        data = sbus_rx.data();
        if (!data.failsafe) {
            processJoystickInput();
        } else {
            stopMotors();
            Serial.println("Power off (failsafe)");
        }
    }
    delay(10); 
}

void processJoystickInput() {
    // Отримуємо значення джойстиків
    joystick_left_y = data.ch[2];
    joystick_left_y_speed = map(joystick_left_y, 173, 1810, 0, 255);
    joystick_right_x = data.ch[0];
    joystick_right_x_speed = map(joystick_right_x, 173, 1810, -255, 255);

    // Перевіряємо управління рухом
    if (data.ch[6] < 990) {
        moveForward();
    } else if (data.ch[6] > 1000) {
        moveBackward();
    } else {
        stopMotors();
        Serial.println("Neutral");
    }

    // Перемикання реле 1 (вгору/вниз)
    bool currentSwitchStateRelay1 = (data.ch[4] > 993); // Верхнє положення
    toggleRelay(IN1, 0, currentSwitchStateRelay1, previousSwitchStateRelay1);

    // Перемикання реле 2 (менше 993)
    bool currentSwitchStateRelay2 = (data.ch[4] < 993); // Нижнє положення
    toggleRelay(IN2, 1, currentSwitchStateRelay2, previousSwitchStateRelay2);

    // Виведення дебаг-інформації
    Serial.print("Speed: ");
    Serial.print(joystick_left_y_speed);
    Serial.print(" Turn: ");
    Serial.println(joystick_right_x_speed);
}

void moveForward() {
    setMotorSpeeds(joystick_left_y_speed, joystick_right_x_speed);
    setMotorDirections(false, true); // Motor A: Forward, Motor B: Forward
    Serial.println("Move forward");
}

void moveBackward() {
    setMotorSpeeds(joystick_left_y_speed, joystick_right_x_speed);
    setMotorDirections(true, false); // Motor A: Backward, Motor B: Backward
    Serial.println("Move backward");
}

void stopMotors() {
    digitalWrite(MOTORA_1, LOW);
    digitalWrite(MOTORA_2, LOW);
    digitalWrite(MOTORB_1, LOW);
    digitalWrite(MOTORB_2, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void setMotorSpeeds(int baseSpeed, int turnAdjustment) {
    int leftSpeed = constrain(baseSpeed + turnAdjustment, 0, 255);
    int rightSpeed = constrain(baseSpeed - turnAdjustment, 0, 255);

    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
}

void setMotorDirections(bool motorA_reverse, bool motorB_reverse) {
    digitalWrite(MOTORA_1, motorA_reverse);
    digitalWrite(MOTORA_2, !motorA_reverse);
    digitalWrite(MOTORB_1, motorB_reverse);
    digitalWrite(MOTORB_2, !motorB_reverse);
}

void relay_init(void) {  
    //set all the relays OUTPUT
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(IN7, OUTPUT);
    pinMode(IN8, OUTPUT);
    
    relay_SetStatus(OFF, OFF);  //turn off all the relay
}

void relay_SetStatus(int relayPin, unsigned char status) {
    digitalWrite(relayPin, status);
}

void toggleRelay(int relayPin, int relayIndex, bool currentSwitchState, bool& previousSwitchState) {
    // Перемикаємо реле тільки при зміні стану перемикача
    if (currentSwitchState && !previousSwitchState) {
        if (relayIndex == 0) {
            relay1State = !relay1State; // Перемикаємо стан реле 1
            relay_SetStatus(relayPin, relay1State ? ON : OFF);
            Serial.print("Relay 1 ");
            Serial.println(relay1State ? "ON" : "OFF");
        } else if (relayIndex == 1) {
            relay2State = !relay2State; // Перемикаємо стан реле 2
            relay_SetStatus(relayPin, relay2State ? ON : OFF);
            Serial.print("Relay 2 ");
            Serial.println(relay2State ? "ON" : "OFF");
        }
    }
    previousSwitchState = currentSwitchState; // Оновлюємо попередній стан перемикача
}
