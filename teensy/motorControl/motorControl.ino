#include <FlexCAN_T4.h>

#define HEADER 0xAA
#define NUM_MOTORS 1
#define PACKET_SIZE (1 + 4*NUM_MOTORS)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

constexpr uint32_t DT_US = 500;  // 2 kHz
static uint32_t last = 0;

struct __attribute__((packed)) MotorStatus1 {
    int8_t   motorTemp;
    int8_t   mosTemp;
    uint8_t  brakeReleased;
    uint16_t voltage;     // 0.1V/LSB
    uint16_t errorFlags;
};

struct __attribute__((packed)) MotorStatus2 {
    int8_t  motorTemp;
    int16_t iq;      // 0.01A/LSB
    int16_t speed;   // 1 dps/LSB
    int16_t angle;   // 1 degree/LSB
};


struct __attribute__((packed)) MotorStatus3 {
    int8_t  motorTemp;
    int16_t iA;
    int16_t iB;
    int16_t iC;
};


const uint32_t STREAM_SYNC = 0xAABBCCDD;

uint8_t rxBuf[PACKET_SIZE];
int rxIndex = 0;
float motorCmd[NUM_MOTORS];
uint8_t motorIDs[NUM_MOTORS] = {3};
MotorStatus2 motorData[NUM_MOTORS];


void handleSerialInput() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (rxIndex == 0) {
      if (b != HEADER) continue;
    }

    rxBuf[rxIndex++] = b;

    if (rxIndex >= PACKET_SIZE) {
      noInterrupts();
      for (int i = 0; i < NUM_MOTORS; i++) {
        float val;
        memcpy(&val, &rxBuf[1 + 4*i], 4);
        motorCmd[i] = val;
      }
      interrupts();

      rxIndex = 0;
    }
  }
}

int motorIndexFromID(uint8_t id) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorIDs[i] == id) return i;
  }
  return -1;
}

void processCANRx(bool receivedConfirm[NUM_MOTORS]) {
  CAN_message_t rx;
  MotorStatus2 status;

  while (Can1.read(rx)) {
    //Serial.println('6');
    if (canParseTorqueControlReply(rx, status)) {
      int idx = motorIndexFromID(rx.id-0x240);
      //Serial.println(idx);
      if (idx >= 0) {
        motorData[idx] = status;
        receivedConfirm[idx] = true;
      }
    }
  }
}



void setup() {
  Serial.begin(460800);

  Can1.begin();
  Can1.setBaudRate(1000000);

  Serial.println();
  Serial.println("Program Start");
}


void loop() {

  //Serial.println(sizeof(MotorStatus2));

  uint32_t now = micros();
  if (now - last < DT_US) return; 
  last += DT_US;

  handleSerialInput();

  for (int i = 0; i < NUM_MOTORS; i++) {
    canTorqueControl(motorIDs[i], motorCmd[i] * 100.0f);
  }

  bool receivedConfirm[NUM_MOTORS] = {};   // all false
  processCANRx(receivedConfirm);

  // Build validity mask
  uint8_t validMask = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (receivedConfirm[i])
      validMask |= (1 << i);
  }

  // If nothing fresh, don’t send anything
  if (validMask == 0) {
    //Serial.println('7');
    return;
  }
  
  

  uint32_t ts = micros();

  Serial.write((uint8_t*)&STREAM_SYNC, sizeof(STREAM_SYNC));
  Serial.write((uint8_t*)&ts, sizeof(ts));
  Serial.write(&validMask, sizeof(validMask));

  // Send only fresh motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (receivedConfirm[i]) {
      Serial.write((uint8_t*)&motorData[i], sizeof(MotorStatus2));
    }
  }
}