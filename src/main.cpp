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
void ResetMotor();
void ResetEnable();
void translateIR();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // setup RF24
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();

  // Setup pin mode
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

  for(uint8_t i=0; i<4; i++)
    mEnData[i] = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  ResetMotor();

  // Receive Infrared remote code v4
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
    translateIR();
    IrReceiver.resume(); // Enable receiving of the next value
  }

  if (foundBtn == 1) {
    if (stateBtn != 0 || stateBtn != 255) {
      /* update mCmdData */
      mCmdData = mCmdData | stateBtn;

      /* update mEnData */ // Edit motor speed here
      if (stateBtn == 1 || stateBtn == 2) {
        ResetEnable();
        mEnData[0] = 255;
      }
      if (stateBtn == 4 || stateBtn == 8) {
        ResetEnable();
        mEnData[1] = 255;
      }
      if (stateBtn == 16 || stateBtn == 32) {
        ResetEnable();
        mEnData[2] = 255;
      }
      if (stateBtn == 64 || stateBtn == 128) {
        ResetEnable();
        mEnData[3] = 255;
      }
    }
  } else {
    ResetMotor();
    ResetEnable();
  }


  /* Send Command to Motor */
  digitalWrite(M1L_PIN, (mCmdData & (1 << 0)) >> 0);
  digitalWrite(M1R_PIN, (mCmdData & (1 << 1)) >> 1);
  digitalWrite(M2L_PIN, (mCmdData & (1 << 2)) >> 2);
  digitalWrite(M2R_PIN, (mCmdData & (1 << 3)) >> 3);
  digitalWrite(M3L_PIN, (mCmdData & (1 << 4)) >> 4);
  digitalWrite(M3R_PIN, (mCmdData & (1 << 5)) >> 5);
  digitalWrite(M4L_PIN, (mCmdData & (1 << 6)) >> 6);
  digitalWrite(M4R_PIN, (mCmdData & (1 << 7)) >> 7);

  /* Send Enable to Motor */
  analogWrite(M1EN_PIN, mEnData[0]);
  analogWrite(M2EN_PIN, mEnData[1]);
  analogWrite(M3EN_PIN, mEnData[2]);
  analogWrite(M4EN_PIN, mEnData[3]);

  foundBtn = 0;
  delay(100);
}

void ResetMotor(){
  mCmdData = 0x0;
}

void ResetEnable(){
  for (uint8_t i = 0; i<4; i++)
    mEnData[i] = 0;
}

void translateIR(){
  uint8_t isRepeat = 0;
  uint8_t newBtn = 0;

  switch(IrReceiver.decodedIRData.decodedRawData) {
    case 0xF30CFF00: // Motor 1 Left
      newBtn = 1;
      break;
    case 0xE718FF00: // Motor 1 Right
      newBtn = 2;
      break;
    case 0xA15EFF00: // Motor 2 Left
      newBtn = 4;
      break;
    case 0xF708FF00: // Motor 2 Right
      newBtn = 8;
      break;
    case 0xE31CFF00: // Motor 3 Left
      newBtn = 16;
      break;
    case 0xA55AFF00: // Motor 3 Right
      newBtn = 32;
      break;
    case 0xAD52FF00: // Motor 4 Left
      newBtn = 64;
      break;
    case 0xBD42FF00: // Motor 4 Right
      newBtn = 128;
      break;
    case 0x0:
      isRepeat = 1;
      break;
    default:
      Serial.println("Unused Btn");
      newBtn = 255;
      break;
  }

  if (newBtn != 0) {
    stateBtn = newBtn;
    btnRepeat = stateBtn;
  } else {
    if (isRepeat)
      stateBtn = btnRepeat;
  }

  if (stateBtn != 0)
    foundBtn = 1;
}
