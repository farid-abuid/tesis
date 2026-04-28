#include <FlexCAN_T4.h>

#define HEADER1 0xAA
#define HEADER2 0x55
#define MAX_MOTORS 8

bool print_diagnostics = true;

const float Kt = 1.9;

const uint8_t torqueLimits[MAX_MOTORS] = {50, 50, 20, 50, 50, 20};

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

struct MotorStatus2;
// motors 1-3 -> Can1, motors 4-6 -> Can2
static inline bool useCan2(uint8_t motorID) { return motorID >= 4; }

// broadcast helper: send the same frame on both buses
static inline void canWriteBoth(const CAN_message_t &msg)
{
    Can1.write(msg);
    Can2.write(msg);
}

constexpr uint32_t DT_US = 500;
static uint32_t last = 0;

const float max_current = 3.0; // Amps
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


// ===== Diagnostics =====
// Round-trip: first CAN send -> after processCANRx (command batch to feedback ready).
// Per-motor: micros() after canXxx write for that motor -> CAN reply parsed for that idx.
constexpr uint32_t DIAG_DECIMATION = 500;

uint32_t diag_counter = 0;

uint32_t roundtrip_sum_us = 0;
uint32_t roundtrip_max_us = 0;

uint32_t motor_lat_sum_us = 0;
uint32_t motor_lat_max_us = 0;
uint32_t motor_lat_count = 0;

uint32_t loop_overruns = 0;
uint32_t last_loop_time = 0;
uint32_t t_cmd_sent_us[MAX_MOTORS];


//========================

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
        //SerialUSB1.println(id);
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
            //SerialUSB1.println("Packet received");
            
            rxBuf[rxIndex++] = b;

            if (rxIndex == 1)
            {
                cmd_type = (rxBuf[0] >> 4) & 0x0F;
                n_motors = rxBuf[0] & 0x0F;
                //SerialUSB1.println(cmd_type);
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

// Maps serial cmd_type to the CAN reply byte the motor echoes back.
static uint8_t expectedReplyByte(uint8_t type)
{
    switch (type)
    {
    case 1:  return 0xA1; // torque
    case 2:  return 0xA2; // speed
    case 3:  return 0xA9; // position (force position reply)
    case 5:  return 0x9C; // read-only (read status 2)
    case 6:  return 0x81; // stop
    case 7:  return 0x80; // shutdown
    default: return 0x00;
    }
}

bool canParseAnyReply(const CAN_message_t &msg,
                      uint8_t type,
                      MotorStatus2 &out)
{
    if (msg.buf[0] != expectedReplyByte(type))
        return false;

    out.motorID = (uint8_t)(msg.id - 0x240);

    int16_t iq_raw    = (int16_t)(msg.buf[2] | (msg.buf[3] << 8));
    int16_t speed_raw = (int16_t)(msg.buf[4] | (msg.buf[5] << 8));
    int16_t angle_raw = (int16_t)(msg.buf[6] | (msg.buf[7] << 8));

    out.iq    = (float)iq_raw / 100.0f * Kt; // Nm torque
    out.speed = (float)speed_raw * PI / 180.0f; // rad/s 
    out.angle = (float)angle_raw * PI / 180.0f; // rad

    return true;
}

// drain a CAN bus and update motorData for any matching replies
#define DRAIN_BUS(bus)                                             \
    do {                                                           \
        CAN_message_t rx;                                          \
        MotorStatus2 status;                                       \
        while ((bus).read(rx))                                     \
        {                                                          \
            if (canParseAnyReply(rx, cmd_type, status))            \
            {                                                      \
                int idx = motorIndexFromID(status.motorID);        \
                if (idx >= 0)                                      \
                {                                                  \
                    motorData[idx] = status;                       \
                    uint32_t t_now = micros();                     \
                    uint32_t dt = t_now - t_cmd_sent_us[idx];      \
                    motor_lat_sum_us += dt;                        \
                    motor_lat_count++;                             \
                    if (dt > motor_lat_max_us)                     \
                        motor_lat_max_us = dt;                     \
                }                                                  \
            }                                                      \
        }                                                          \
    } while (0)

void processCANRx()
{
    DRAIN_BUS(Can1);
    DRAIN_BUS(Can2);
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

    if (useCan2(motorID))
        Can2.write(msg);
    else
        Can1.write(msg);
}

void canSpeedControl(uint8_t motorID, int32_t speedControl, uint8_t maxTorque)
{
    CAN_message_t msg{};
    msg.id  = motorID + 0x140;
    msg.len = 8;    

    msg.buf[0] = 0xA2;
    msg.buf[1] = maxTorque;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    int32ToBytes(speedControl, &msg.buf[4]);

    if (useCan2(motorID))
        Can2.write(msg);
    else
        Can1.write(msg);
}


void canForcePositionControl(uint8_t motorID,
                             int32_t angleControl,
                             uint16_t maxSpeed,
                             uint8_t maxTorque)
{
    CAN_message_t msg;
    msg.id  = motorID + 0x140;
    msg.len = 8;

    msg.buf[0] = 0xA9;
    msg.buf[1] = maxTorque;

    msg.buf[2] = (uint8_t)(maxSpeed);
    msg.buf[3] = (uint8_t)(maxSpeed >> 8);

    int32ToBytes(angleControl, &msg.buf[4]);

    if (useCan2(motorID))
        Can2.write(msg);
    else
        Can1.write(msg);
}

void canReadOnlyAll()
{
    CAN_message_t msg;
    msg.id  = 0x280;
    msg.len = 8;

    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x9C;
    canWriteBoth(msg);
}

void canSetZeroPositionAll()
{
    CAN_message_t msg;
    msg.id  = 0x280;
    msg.len = 8;

    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x64;
    canWriteBoth(msg);
}

void canSystemResetAll()
{
    CAN_message_t msg;
    msg.id  = 0x280;
    msg.len = 8;

    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x76;
    canWriteBoth(msg);
}

void canStopAll()
{
    CAN_message_t msg;
    msg.id  = 0x280;
    msg.len = 8;

    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x81;
    canWriteBoth(msg);
}

void canShutdownAll()
{
    CAN_message_t msg;
    msg.id  = 0x280;
    msg.len = 8;

    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x80;
    canWriteBoth(msg);
}


void canSystemReleaseBreakAll()
{
    CAN_message_t msg;
    msg.id  = 0x280;
    msg.len = 8;

    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x77;
    canWriteBoth(msg);
}

void canSystemCloseBreakAll()
{
    CAN_message_t msg;
    msg.id  = 0x280;
    msg.len = 8;

    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x78;
    canWriteBoth(msg);
}


void setup()
{
    Serial.begin(460800);     // telemetry
    SerialUSB1.begin(460800); // diagnostics

    Can1.begin();
    Can1.setBaudRate(1000000);

    Can2.begin();
    Can2.setBaudRate(1000000);

    //SerialUSB1.println();
    //SerialUSB1.println("Program Start");
    canSetZeroPositionAll();
    delay(50);
    canSystemResetAll();
    delay(50);
}

void loop()
{
    uint32_t now = micros();

    uint32_t loop_dt = now - last;

    if (loop_dt < DT_US) return;

    if (loop_dt > DT_US)
        loop_overruns++;  // track missed deadlines

    last += DT_US;
    last_loop_time = loop_dt;

    handleSerialInput();
    //SerialUSB1.println(n_motors);
    if (!send_command)
        return;

    uint32_t t_cycle_start = micros();
 
    switch (cmd_type)
    {

    case 1:

        for (int i = 0; i < n_motors; i++)
        {
            float iq_A = motorCmd[i] / Kt;  // Nm -> A (float)
            iq_A = min(max(iq_A, -max_current), max_current);
            int16_t iq = (int16_t)(iq_A * 100.0f);  // convert to CAN units
            canTorqueControl(motorIDs[i], iq);

            t_cmd_sent_us[i] = micros();
        }

        break;

    case 2:

        for (int i = 0; i < n_motors; i++)
        {
            int32_t speed = motorCmd[i] * 180.0f / PI * 100.0f; //rad/s -> deg/s -> 1000*deg/s (CAN)
            //uint8_t maxTorque = 25; // (0-100)% of rated current. If 0, limit is stall current (21 A)
            canSpeedControl(motorIDs[i], speed, torqueLimits[i]);
            
            t_cmd_sent_us[i] = micros();
        }

        break;

    case 3:

        for (int i = 0; i < n_motors; i++)
        {
            uint16_t maxSpeed = 400; // % dps, almost top speed 
            //uint8_t maxTorque = 25; // (0-100)% of rated current. If 0, limit is stall current (21 A)
            int32_t angle = motorCmd[i] * 180.0f / PI * 100.0f; //rad -> deg -> 100*deg (CAN)
            canForcePositionControl(motorIDs[i], angle, maxSpeed, torqueLimits[i]);
        
            t_cmd_sent_us[i] = micros();
        }
        break;

    case 5:
        canReadOnlyAll();
        t_cmd_sent_us[0] = micros();
        break;

    case 6:
        canStopAll();
        t_cmd_sent_us[0] = micros();
        break;

    case 7:
        canShutdownAll();
        t_cmd_sent_us[0] = micros();
        break;

    default:
        break;
    }
    
    processCANRx();

    uint32_t rt_us = micros() - t_cycle_start;
    roundtrip_sum_us += rt_us;
    if (rt_us > roundtrip_max_us)
        roundtrip_max_us = rt_us;

    Serial.write(HEADER1);
    Serial.write(HEADER2);

    float iqSum = 0;
    for (int i = 0; i < n_motors; i++)
    {
        Serial.write((uint8_t*)&motorData[i], sizeof(MotorStatus2));
        //SerialUSB1.print("ID=");
        //SerialUSB1.println(motorData[i].motorID);
        //SerialUSB1.print("angle=");
        SerialUSB1.println(motorData[i].angle);
        iqSum += (motorData[i].iq / Kt);
    }


    diag_counter++;

    if (diag_counter >= DIAG_DECIMATION && print_diagnostics == true)
    {
        SerialUSB1.print("iqTotal=");
        SerialUSB1.println(iqSum);
        /*
        uint32_t rt_avg_us = roundtrip_sum_us / DIAG_DECIMATION;
        uint32_t motor_avg_us = (motor_lat_count > 0) ? (motor_lat_sum_us / motor_lat_count) : 0;

        SerialUSB1.print("roundtrip_us avg:");
        SerialUSB1.print(rt_avg_us);
        SerialUSB1.print(" max:");
        SerialUSB1.print(roundtrip_max_us);
        SerialUSB1.print(" | motor_CAN_us avg:");
        SerialUSB1.print(motor_avg_us);
        SerialUSB1.print(" max:");
        SerialUSB1.print(motor_lat_max_us);
        SerialUSB1.print(" n:");
        SerialUSB1.print(motor_lat_count);
        SerialUSB1.print(" | loop_dt_us:");
        SerialUSB1.print(last_loop_time);
        SerialUSB1.print(" overruns:");
        SerialUSB1.println(loop_overruns);
        */
        diag_counter = 0;
        roundtrip_sum_us = 0;
        roundtrip_max_us = 0;
        motor_lat_sum_us = 0;
        motor_lat_max_us = 0;
        motor_lat_count = 0;
        loop_overruns = 0;
        
    }
    
    send_command = false;
}