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
int joystick_left_x = 0;
int joystick_left_x_turn = 0;
int speedAdjustment = 0;

bool relay1State = false; // Стан реле 1 (вимкнене за замовчуванням)
bool relay2State = false; // Стан реле 2 (вимкнене за замовчуванням)
bool previousSwitchStateRelay1 = false; // Попередній стан для реле 1
bool previousSwitchStateRelay2 = false;

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
    joystick_left_y_speed = map(joystick_left_y, 173, 1810, 0, 85);
    joystick_right_x = data.ch[0];
    joystick_right_x_speed = map(joystick_right_x, 173, 1810, -85, 85);
    joystick_left_x = data.ch[3];
    joystick_left_x_turn = map(joystick_left_x, 173, 1810, -80, 80);

    // Перевіряємо перемикач B для визначення напрямку руху
    if (data.ch[6] < 990) {
        moveForward();
    } else if (data.ch[6] > 1000) {
        moveBackward();
    } else {
        stopMotors();
        Serial.println("Neutral");
    }

    // Перевіряємо перемикач C для зміни передачі
    if (data.ch[7] == 173) {
        speedAdjustment = 3; // Gear 1
    } else if (data.ch[7] == 993) {
        speedAdjustment = 2; // Gear 2
    } else if (data.ch[7] == 1810) {
        speedAdjustment = 1; // Gear 3
    }

    if (joystick_right_x_speed > 0) {
      turnOnSpot(true);
    } else if (joystick_right_x_speed < 0) {
      turnOnSpot(false);
    }


    bool currentSwitchStateRelay1 = (data.ch[4] > 993); // Верхнє положення
    toggleRelay(IN1, 0, currentSwitchStateRelay1, previousSwitchStateRelay1);

    // Перемикання реле 2 (менше 993)
    bool currentSwitchStateRelay2 = (data.ch[4] < 993); // Нижнє положення
    toggleRelay(IN2, 1, currentSwitchStateRelay2, previousSwitchStateRelay2);


    // Виведення дебаг-інформації
    Serial.print("Speed: ");
    Serial.print(joystick_left_y_speed);
    Serial.print(" Turn (on spot): ");
    Serial.print(joystick_right_x_speed);
    Serial.print(" Turn (while driving): ");
    Serial.println(joystick_left_x_turn);
}

void moveForward() {
    int adjustedSpeedLeft = joystick_left_y_speed * speedAdjustment - joystick_left_x_turn;
    int adjustedSpeedRight = joystick_left_y_speed * speedAdjustment + joystick_left_x_turn;

    setMotorSpeeds(adjustedSpeedLeft, adjustedSpeedRight);
    setMotorDirections(false, true);
    Serial.println("Move forward");
}

void moveBackward() {
    int adjustedSpeedLeft = joystick_left_y_speed * speedAdjustment - joystick_left_x_turn;
    int adjustedSpeedRight = joystick_left_y_speed * speedAdjustment + joystick_left_x_turn;

    setMotorSpeeds(adjustedSpeedLeft, adjustedSpeedRight);
    setMotorDirections(true, false);
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

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
}

void setMotorDirections(bool motorA_reverse, bool motorB_reverse) {
    digitalWrite(MOTORA_1, motorA_reverse);
    digitalWrite(MOTORA_2, !motorA_reverse);
    digitalWrite(MOTORB_1, motorB_reverse);
    digitalWrite(MOTORB_2, !motorB_reverse);
}

void turnOnSpot(bool clockwise) {
    int turnSpeed = abs(joystick_right_x_speed) * speedAdjustment;
    if (clockwise) {
        analogWrite(ENA, turnSpeed);
        analogWrite(ENB, turnSpeed);
        digitalWrite(MOTORA_1, LOW);
        digitalWrite(MOTORA_2, HIGH); // Ліва гусениця вперед
        digitalWrite(MOTORB_1, LOW);
        digitalWrite(MOTORB_2, HIGH);
    } else {
        analogWrite(ENA, turnSpeed);
        analogWrite(ENB, turnSpeed);
        digitalWrite(MOTORA_1, HIGH);
        digitalWrite(MOTORA_2, LOW); // Ліва гусениця назад
        digitalWrite(MOTORB_1, HIGH);
        digitalWrite(MOTORB_2, LOW);
    }
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

