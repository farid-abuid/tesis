// CAN HELPER FUNCTIONS

#include <FlexCAN_T4.h>

//FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

static inline uint16_t motorCmdID(uint8_t motorID) {
    return 0x140 + motorID;   // motorID = 1..32
}

static inline uint16_t motorReplyID(uint8_t motorID) {
    return 0x240 + motorID;
}

static inline void packFloatLE(float value, uint8_t *buf) {
    union {
        float f;
        uint32_t u;
    } v;
    v.f = value;
    buf[0] = (uint8_t)(v.u);
    buf[1] = (uint8_t)(v.u >> 8);
    buf[2] = (uint8_t)(v.u >> 16);
    buf[3] = (uint8_t)(v.u >> 24);
}

static inline float unpackFloatLE(const uint8_t *buf) {
    union {
        float f;
        uint32_t u;
    } v;
    v.u =  (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
    return v.f;
}

static inline int32_t unpackInt32LE(const uint8_t *buf) {
    return (int32_t)(
          (uint32_t)buf[0]
        | ((uint32_t)buf[1] << 8)
        | ((uint32_t)buf[2] << 16)
        | ((uint32_t)buf[3] << 24)
    );
}

static inline int32_t bytesToInt32(const uint8_t *b) {
    return (int32_t)(
        ((uint32_t)b[0]) |
        ((uint32_t)b[1] << 8) |
        ((uint32_t)b[2] << 16) |
        ((uint32_t)b[3] << 24)
    );
}

static inline void int32ToBytes(int32_t v, uint8_t *b) {
    b[0] = (uint8_t)(v);
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

// ------------------------------------COMMANDS FROM CAN MANUAL--------------------------------------------

void canReadPID(uint8_t motorID, uint8_t index) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x30;
    msg.buf[1] = index;
    for (int i = 2; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

bool canParsePIDReply(const CAN_message_t &msg,
                      uint8_t motorID,
                      uint8_t expectedIndex,
                      float &valueOut)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0x30) return false;
    if (msg.buf[1] != expectedIndex) return false;

    valueOut = unpackFloatLE(&msg.buf[4]);
    return true;
}

void canWritePID_RAM(uint8_t motorID, uint8_t index, float value) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x31;
    msg.buf[1] = index;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    packFloatLE(value, &msg.buf[4]);

    Can1.write(msg);
}

void canReadAcceleration(uint8_t motorID, uint8_t index) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x42;
    msg.buf[1] = index;
    for (int i = 2; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

bool canParseAccelerationReply(const CAN_message_t &msg,
                               uint8_t motorID,
                               uint8_t expectedIndex,
                               int32_t &accelOut)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0x42) return false;
    if (msg.buf[1] != expectedIndex) return false;

    accelOut = unpackInt32LE(&msg.buf[4]);
    return true;
}


void canWriteAcceleration(uint8_t motorID, uint8_t index, uint32_t accel) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x43;
    msg.buf[1] = index;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    msg.buf[4] = (uint8_t)(accel);
    msg.buf[5] = (uint8_t)(accel >> 8);
    msg.buf[6] = (uint8_t)(accel >> 16);
    msg.buf[7] = (uint8_t)(accel >> 24);

    Can1.write(msg);
}

void canReadEncoderMultiTurn(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = 0x140 + motorID;
    msg.len = 8;
    msg.buf[0] = 0x60;
    Can1.write(msg);
}

bool canParseEncoderMultiTurn(const CAN_message_t &rx,
                              uint8_t motorID,
                              int32_t &encoder)
{
    if (rx.id != (uint32_t)(0x240u + motorID)) return false;
 return false;
    if (rx.buf[0] != 0x60) return false;

    encoder = bytesToInt32(&rx.buf[4]);
    return true;
}

void canReadEncoderRaw(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = 0x140 + motorID;
    msg.len = 8;
    msg.buf[0] = 0x61;
    Can1.write(msg);
}

bool canParseEncoderRaw(const CAN_message_t &rx,
                         uint8_t motorID,
                         int32_t &encoderRaw)
{
    if (rx.id != (uint32_t)(0x240u + motorID)) return false;
 return false;
    if (rx.buf[0] != 0x61) return false;

    encoderRaw = bytesToInt32(&rx.buf[4]);
    return true;
}

void canReadEncoderOffset(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = 0x140 + motorID;
    msg.len = 8;
    msg.buf[0] = 0x62;
    Can1.write(msg);
}

bool canParseEncoderOffset(const CAN_message_t &rx,
                            uint8_t motorID,
                            int32_t &offset)
{
    if (rx.id != (uint32_t)(0x240u + motorID)) return false;
 return false;
    if (rx.buf[0] != 0x62) return false;

    offset = bytesToInt32(&rx.buf[4]);
    return true;
}

void canWriteEncoderZero(uint8_t motorID, int32_t offset) {
    CAN_message_t msg{};
    msg.id  = 0x140 + motorID;
    msg.len = 8;
    msg.buf[0] = 0x63;
    int32ToBytes(offset, &msg.buf[4]);
    Can1.write(msg);
}

bool canParseWriteEncoderZeroAck(const CAN_message_t &rx,
                                 uint8_t motorID)
{
    return (rx.id == (0x240 + motorID)) && (rx.buf[0] == 0x63);
}

void canWriteCurrentPosAsZero(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = 0x140 + motorID;
    msg.len = 8;
    msg.buf[0] = 0x64;
    Can1.write(msg);
}

bool canParseCurrentPosAsZeroReply(const CAN_message_t &rx,
                                   uint8_t motorID,
                                   int32_t &newOffset)
{
    if (rx.id != (uint32_t)(0x240u + motorID)) return false;
 return false;
    if (rx.buf[0] != 0x64) return false;

    newOffset = bytesToInt32(&rx.buf[4]);
    return true;
}

void canReadSingleTurnEncoder(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = 0x140 + motorID;
    msg.len = 8;
    msg.buf[0] = 0x90;
    Can1.write(msg);
}

bool canParseSingleTurnEncoder(const CAN_message_t &rx,
                               uint8_t motorID,
                               uint16_t &encoder,
                               uint16_t &encoderRaw,
                               uint16_t &encoderOffset)
{
    if (rx.id != (uint32_t)(0x240u + motorID)) return false;
 return false;
    if (rx.buf[0] != 0x90) return false;

    encoder       = (uint16_t)(rx.buf[2] | (rx.buf[3] << 8));
    encoderRaw    = (uint16_t)(rx.buf[4] | (rx.buf[5] << 8));
    encoderOffset = (uint16_t)(rx.buf[6] | (rx.buf[7] << 8));

    return true;
}

void canReadMultiTurnAngle(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x92;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

bool canParseMultiTurnAngleReply(const CAN_message_t &msg,
                                 uint8_t motorID,
                                 int32_t &angleOut)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0x92) return false;

    angleOut = unpackInt32LE(&msg.buf[4]);
    return true;
}

void canReadSingleTurnAngle(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x94;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

bool canParseSingleTurnAngleReply(const CAN_message_t &msg,
                                  uint8_t motorID,
                                  uint16_t &angleOut)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0x94) return false;

    angleOut = (uint16_t)(
        ((uint16_t)msg.buf[6]) |
        ((uint16_t)msg.buf[7] << 8)
    );
    return true;
}


void canReadMotorStatus1(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x9A;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

bool canParseMotorStatus1Reply(const CAN_message_t &msg,
                               uint8_t motorID,
                               MotorStatus1 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0x9A) return false;

    out.motorTemp     = (int8_t)msg.buf[1];
    out.mosTemp       = (int8_t)msg.buf[2];
    out.brakeReleased = msg.buf[3];
    out.voltage       = (uint16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.errorFlags    = (uint16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}
void canReadMotorStatus2(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x9C;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

bool canParseMotorStatus2Reply(const CAN_message_t &msg,
                               uint8_t motorID,
                               MotorStatus2 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0x9C) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iq        = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.speed     = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.angle     = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}

void canReadMotorStatus3(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x9D;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

bool canParseMotorStatus3Reply(const CAN_message_t &msg,
                               uint8_t motorID,
                               MotorStatus3 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0x9D) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iA        = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.iB        = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.iC        = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}


void canMotorShutdown() {
    CAN_message_t msg{};
    msg.id  = 0x280;
    msg.len = 8;

    msg.buf[0] = 0x80;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

void canMotorStop(uint8_t motorID) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0x81;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    Can1.write(msg);
}

void canTorqueControl(uint8_t motorID, int16_t iqControl) {
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0xA1;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = (uint8_t)(iqControl);
    msg.buf[5] = (uint8_t)(iqControl >> 8);
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    Can1.write(msg);
}

bool canParseTorqueControlReply(const CAN_message_t &msg,
                                //uint8_t motorID,
                                MotorStatus2 &out)
{
    //if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0xA1) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iq        = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.speed     = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.angle     = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}


void canSpeedControl(uint8_t motorID,
                     int32_t speedControl,
                     uint8_t maxTorque)
{
    CAN_message_t msg{};
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0xA2;
    msg.buf[1] = maxTorque;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    int32ToBytes(speedControl, &msg.buf[4]);

    Can1.write(msg);
}

bool canParseSpeedControlReply(const CAN_message_t &msg,
                               uint8_t motorID,
                               MotorStatus2 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0xA2) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iq        = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.speed     = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.angle     = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}

void canAbsolutePositionControl(uint8_t motorID,
                                int32_t angleControl,
                                uint16_t maxSpeed)
{
    CAN_message_t msg;
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0xA4;
    msg.buf[1] = 0x00;

    msg.buf[2] = (uint8_t)(maxSpeed);
    msg.buf[3] = (uint8_t)(maxSpeed >> 8);

    msg.buf[4] = (uint8_t)(angleControl);
    msg.buf[5] = (uint8_t)(angleControl >> 8);
    msg.buf[6] = (uint8_t)(angleControl >> 16);
    msg.buf[7] = (uint8_t)(angleControl >> 24);

    Can1.write(msg);
}

bool canParseAbsolutePositionReply(const CAN_message_t &msg,
                                   uint8_t motorID,
                                   MotorStatus2 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0xA4) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iq    = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.speed = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.angle = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}


void canSingleTurnPosition(uint8_t motorID,
                           uint16_t angleControl,
                           uint8_t spinDirection,
                           uint16_t maxSpeed)
{
    CAN_message_t msg;
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0xA6;
    msg.buf[1] = spinDirection;

    msg.buf[2] = (uint8_t)(maxSpeed);
    msg.buf[3] = (uint8_t)(maxSpeed >> 8);

    msg.buf[4] = (uint8_t)(angleControl);
    msg.buf[5] = (uint8_t)(angleControl >> 8);

    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    Can1.write(msg);
}

bool canParseSingleTurnReply(const CAN_message_t &msg,
                             uint8_t motorID,
                             MotorStatus2 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0xA6) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iq      = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.speed   = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.angle = (uint16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}


void canIncrementalPosition(uint8_t motorID,
                            int32_t angleIncrement,
                            uint16_t maxSpeed)
{
    CAN_message_t msg;
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0xA8;
    msg.buf[1] = 0x00;

    msg.buf[2] = (uint8_t)(maxSpeed);
    msg.buf[3] = (uint8_t)(maxSpeed >> 8);

    msg.buf[4] = (uint8_t)(angleIncrement);
    msg.buf[5] = (uint8_t)(angleIncrement >> 8);
    msg.buf[6] = (uint8_t)(angleIncrement >> 16);
    msg.buf[7] = (uint8_t)(angleIncrement >> 24);

    Can1.write(msg);
}

bool canParseIncrementalReply(const CAN_message_t &msg,
                              uint8_t motorID,
                              MotorStatus2 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0xA8) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iq    = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.speed = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.angle = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}


void canForcePositionControl(uint8_t motorID,
                             int32_t angleControl,
                             uint16_t maxSpeed,
                             uint8_t maxTorque)
{
    CAN_message_t msg;
    msg.id  = motorCmdID(motorID);
    msg.len = 8;

    msg.buf[0] = 0xA9;
    msg.buf[1] = maxTorque;

    msg.buf[2] = (uint8_t)(maxSpeed);
    msg.buf[3] = (uint8_t)(maxSpeed >> 8);

    msg.buf[4] = (uint8_t)(angleControl);
    msg.buf[5] = (uint8_t)(angleControl >> 8);
    msg.buf[6] = (uint8_t)(angleControl >> 16);
    msg.buf[7] = (uint8_t)(angleControl >> 24);

    Can1.write(msg);
}

bool canParseForcePositionReply(const CAN_message_t &msg,
                                uint8_t motorID,
                                MotorStatus2 &out)
{
    if (msg.id != motorReplyID(motorID)) return false;
    if (msg.buf[0] != 0xA9) return false;

    out.motorTemp = (int8_t)msg.buf[1];
    out.iq    = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    out.speed = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    out.angle = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    return true;
}

