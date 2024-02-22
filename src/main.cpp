#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 15
#define CE_PIN 9
#define CNS_PIN 10

// Motor Direction Control Scope
#define M1L_PIN A0
#define M1R_PIN A1
#define M2L_PIN A2
#define M2R_PIN A3
#define M3L_PIN 22
#define M3R_PIN 24
#define M4L_PIN 23
#define M4R_PIN 25

// Motor Enable Control Scope
#define M1EN_PIN 6
#define M2EN_PIN 7
#define M3EN_PIN 4
#define M4EN_PIN 5

// Motor Command Data
uint8_t mCmdData = 0x0;

/* motor enable data */
uint8_t mEnData[4];

uint8_t stateBtn;
uint8_t btnRepeat;
uint8_t foundBtn;

/* Infrared Receiver Scope */
unsigned long key_value = 0;

// RF24 pipe address
const byte thisSlaveAddress[5] = {'S','s','T','M','U'};

RF24 radio(CE_PIN, CNS_PIN);

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  pinMode(M1L_PIN, OUTPUT);
  pinMode(M1R_PIN, OUTPUT);
  pinMode(M2L_PIN, OUTPUT);
  pinMode(M2R_PIN, OUTPUT);
  pinMode(M3L_PIN, OUTPUT);
  pinMode(M3R_PIN, OUTPUT);
  pinMode(M4L_PIN, OUTPUT);
  pinMode(M4R_PIN, OUTPUT);
  pinMode(M1EN_PIN, OUTPUT);
  pinMode(M2EN_PIN, OUTPUT);
  pinMode(M3EN_PIN, OUTPUT);
  pinMode(M4EN_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
}
