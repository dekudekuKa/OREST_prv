#include "sbus.h"
#include "Servo.h"

#define ENA 3      //ENA
#define MOTORA_1 4 //IN3
#define MOTORA_2 5 //IN4
#define MOTORB_1 8 //IN1
#define MOTORB_2 7 //IN2
#define ENB 6      //ENB

bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusTx sbus_tx(&Serial1); 
bfs::SbusData data;

int joystic_r_x_angle = 0;
int joystic_r_y_angle = 0;
int joystick_l_x_angle = 0;
int joystick_l_y_speed = 0;

int joystick_l_y = 0;

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

    digitalWrite(ENA,0);
    digitalWrite(ENB,0);

}

void loop() {
    if (sbus_rx.Read()) {
        data = sbus_rx.data();
        // for (int8_t i = 0; i < data.NUM_CH; i++) {
        //   Serial.print(data.ch[i]);
        //   Serial.print(":");
        //   Serial.print(i);
        //   Serial.print("\t");
        // }

        // Serial.print(data.lost_frame);
        // Serial.print("\t");
        // Serial.println(data.failsafe);
        // Serial.print("\t");

        // sbus_tx.data(data);

        // sbus_tx.Write();

    joystick_l_y = data.ch[2];
    joystick_l_y_speed = map(joystick_l_y, 173, 1810, 0, 255);

    if (data.ch[6] < 990) {
        digitalWrite(MOTORA_1, HIGH);
        digitalWrite(MOTORA_2, LOW);
        digitalWrite(MOTORB_1, LOW);
        digitalWrite(MOTORB_2, HIGH);
        analogWrite(ENB, joystick_l_y_speed);
        analogWrite(ENA, joystick_l_y_speed);
        Serial.println("Move forward");
    } else if (data.ch[6] > 1000) {
        digitalWrite(MOTORA_1, LOW);
        digitalWrite(MOTORA_2, HIGH);
        digitalWrite(MOTORB_1, HIGH);
        digitalWrite(MOTORB_2, LOW);
        analogWrite(ENB, joystick_l_y_speed);
        analogWrite(ENA, joystick_l_y_speed);         
        Serial.println("Move backward");
    }else{
        digitalWrite(MOTORA_1, LOW);
        digitalWrite(MOTORA_2, LOW);
        digitalWrite(MOTORB_1, LOW);
        digitalWrite(MOTORB_2, LOW);
        Serial.println("Neutral");
    }
    Serial.println(joystick_l_y_speed);

      }
}