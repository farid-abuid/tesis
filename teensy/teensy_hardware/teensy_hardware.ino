#include <FlexCAN_T4.h>

#define HEADER1 0xAA
#define HEADER2 0x55
#define MAX_MOTORS 8

const float Kt = 1.9;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

constexpr uint32_t DT_US = 500;
static uint32_t last = 0;

bool send_command = false;
uint8_t cmd_type = 0;
uint8_t n_motors = 0;

struct __attribute__((packed)) MotorStatus2 {
    uint8_t  motorID;
    float iq;
    float speed;
    float angle;
};

float motorCmd[MAX_MOTORS];
uint8_t motorIDs[MAX_MOTORS];
MotorStatus2 motorData[MAX_MOTORS];

uint8_t rxBuf[64];

enum ParserState
{
    WAIT_HEADER1,
    WAIT_HEADER2,
    READ_PACKET
};

ParserState parserState = WAIT_HEADER1;

int rxIndex = 0;
int expectedLength = 0;

static inline uint8_t computeChecksum(uint8_t *data, int len)
{
    uint8_t cs = 0;

    for (int i = 0; i < len; i++)
        cs ^= data[i];

    return cs;
}

void parseMotorCommands()
{
    for (int i = 0; i < n_motors; i++)
    {
        uint8_t id;
        float val;

        memcpy(&id, &rxBuf[1 + 5*i], 1);
        memcpy(&val, &rxBuf[2 + 5*i], 4);

        motorIDs[i] = id;
        motorCmd[i] = val;
    }
}

void handleSerialInput()
{
    while (Serial.available())
    {
        uint8_t b = Serial.read();

        switch(parserState)
        {

        case WAIT_HEADER1:

            if (b == HEADER1)
                parserState = WAIT_HEADER2;

            break;

        case WAIT_HEADER2:

            if (b == HEADER2)
            {
                parserState = READ_PACKET;
                rxIndex = 0;
            }
            else
            {
                parserState = WAIT_HEADER1;
            }

            break;

        case READ_PACKET:

            rxBuf[rxIndex++] = b;

            if (rxIndex == 1)
            {
                cmd_type = (rxBuf[0] >> 4) & 0x0F;
                n_motors = rxBuf[0] & 0x0F;

                if (n_motors > MAX_MOTORS)
                {
                    parserState = WAIT_HEADER1;
                    break;
                }

                expectedLength = 1 + n_motors*5 + 1;
            }

            if (rxIndex >= expectedLength)
            {
                uint8_t cs = computeChecksum(rxBuf, expectedLength-1);

                if (cs == rxBuf[expectedLength-1])
                {
                    parseMotorCommands();
                    send_command = true;
                }

                parserState = WAIT_HEADER1;
            }

            break;
        }
    }
}

int motorIndexFromID(uint8_t id)
{
    for (int i = 0; i < n_motors; i++)
        if (motorIDs[i] == id)
            return i;

    return -1;
}

static inline void int32ToBytes(int32_t v, uint8_t *b)
{
    b[0] = (uint8_t)(v);
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

bool canParseAnyReply(const CAN_message_t &msg,
                      uint8_t type,
                      MotorStatus2 &out)
{
    if (msg.buf[0] != 0xA0 + type)
        return false;

    out.motorID = (uint8_t)(msg.id - 0x240);

    int16_t iq_raw    = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    int16_t speed_raw = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    int16_t angle_raw = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    out.iq    = (float)iq_raw / 100.0f * Kt; // Already sends Nm torque
    out.speed = (float)speed_raw * PI / 180.0f;
    out.angle = (float)angle_raw * PI / 180.0f;

    return true;
}

void processCANRx()
{
    CAN_message_t rx;
    MotorStatus2 status;

    while (Can1.read(rx))
    {
        if (canParseAnyReply(rx, cmd_type, status))
        {
            int idx = motorIndexFromID(status.motorID);

            if (idx >= 0)
                motorData[idx] = status;
        }
    }
}

void canTorqueControl(uint8_t motorID, int16_t iqControl)
{
    CAN_message_t msg{};
    msg.id  = motorID + 0x140;
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

void canSpeedControl(uint8_t motorID, int32_t speedControl)
{
    CAN_message_t msg{};
    msg.id  = motorID + 0x140;
    msg.len = 8;

    uint8_t maxTorque = 1;

    msg.buf[0] = 0xA2;
    msg.buf[1] = maxTorque;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    int32ToBytes(speedControl, &msg.buf[4]);

    Can1.write(msg);
}

void canAbsolutePositionControl(uint8_t motorID,
                                int32_t angleControl)
{
    uint16_t maxSpeed = 1;

    CAN_message_t msg{};
    msg.id  = motorID + 0x140;
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

void setup()
{
    Serial.begin(115200);

    Can1.begin();
    Can1.setBaudRate(1000000);

    Serial.println();
    Serial.println("Program Start");
}

void loop()
{
    uint32_t now = micros();

    if (now - last < DT_US) return;
    last += DT_US;

    handleSerialInput();

    if (!send_command)
        return;

    switch (cmd_type)
    {

    case 1:

        for (int i = 0; i < n_motors; i++)
        {
            int16_t iq = motorCmd[i] / Kt * 100.0f; //Nm -> A -> 100*A (CAN)
            canTorqueControl(motorIDs[i], iq);
        }

        break;

    case 2:

        for (int i = 0; i < n_motors; i++)
            canSpeedControl(motorIDs[i], motorCmd[i]);

        break;

    case 4:

        for (int i = 0; i < n_motors; i++)
            canAbsolutePositionControl(motorIDs[i], motorCmd[i]);

        break;

    default:
        break;
    }

    processCANRx();

    Serial.write(HEADER1);
    Serial.write(HEADER2);

    for (int i = 0; i < n_motors; i++)
        Serial.write((uint8_t*)&motorData[i], sizeof(MotorStatus2));

    send_command = false;
}