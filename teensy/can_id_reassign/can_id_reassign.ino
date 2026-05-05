/*
 * CAN motor ID discovery and reassignment (Teensy 4.x + FlexCAN_T4).
 * Mirrors teensy_hardware.ino: dual CAN @ 1 Mbps, broadcast read-all 0x9C @ 0x280,
 * command IDs 0x140+N, reply IDs 0x240+N.
 *
 * Serial (USB): 115200 baud. Commands:
 *   s / S — scan buses (readAll = broadcast 0x9C, collect motor IDs)
 *   r <id> <bus> — read CAN ID register via 0x79 (bus 1 or 2)
 * Then follow prompts to pick a discovered motor and set a new CAN ID (1–32) via 0x79 write.
 */

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

static constexpr uint32_t BROADCAST_ID = 0x280;
static constexpr uint8_t CMD_READ_STATUS2 = 0x9C;
static constexpr uint8_t CMD_CAN_ID = 0x79;

static constexpr uint8_t MAX_FOUND = 32;

struct FoundMotor {
    uint8_t motorId;  // 1..32, logical ID (TX = 0x140 + id)
    uint8_t bus;      // 1 = CAN1, 2 = CAN2
};

static FoundMotor g_found[MAX_FOUND];
static uint8_t g_nFound = 0;

static inline uint16_t motorCmdId(uint8_t motorId)
{
    return (uint16_t)(0x140u + motorId);
}

static inline uint16_t motorReplyId(uint8_t motorId)
{
    return (uint16_t)(0x240u + motorId);
}

static void drainBus(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &bus)
{
    CAN_message_t rx;
    while (bus.read(rx)) {
    }
}

static void drainBus2(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &bus)
{
    CAN_message_t rx;
    while (bus.read(rx)) {
    }
}

static void drainBoth()
{
    drainBus(Can1);
    drainBus2(Can2);
}

static void canWriteBoth(const CAN_message_t &msg)
{
    Can1.write(msg);
    Can2.write(msg);
}

// Broadcast read-all motors (same as canReadOnlyAll() in teensy_hardware.ino).
static void readAllBroadcast()
{
    CAN_message_t msg{};
    msg.id = BROADCAST_ID;
    msg.len = 8;
    memset(msg.buf, 0, 8);
    msg.buf[0] = CMD_READ_STATUS2;
    canWriteBoth(msg);
}

static bool alreadyListed(uint8_t motorId, uint8_t bus)
{
    for (uint8_t i = 0; i < g_nFound; i++) {
        if (g_found[i].motorId == motorId && g_found[i].bus == bus)
            return true;
    }
    return false;
}

static void addFound(uint8_t motorId, uint8_t bus)
{
    if (motorId < 1 || motorId > 32)
        return;
    if (g_nFound >= MAX_FOUND)
        return;
    if (alreadyListed(motorId, bus))
        return;
    g_found[g_nFound].motorId = motorId;
    g_found[g_nFound].bus = bus;
    g_nFound++;
}

// Parse status-2 style reply (0x9C in buf[0]) and derive motor ID from arbitration ID.
static bool parseReadAllReply(const CAN_message_t &msg, uint8_t &motorIdOut)
{
    if (msg.len < 8)
        return false;
    if (msg.buf[0] != CMD_READ_STATUS2)
        return false;
    if (msg.id < 0x241 || msg.id > 0x260)  // 0x240 + 1 .. 0x240 + 32
        return false;
    motorIdOut = (uint8_t)(msg.id - 0x240);
    return motorIdOut >= 1 && motorIdOut <= 32;
}

// Scan both buses after a broadcast read-all.
static void scanMotors()
{
    g_nFound = 0;
    drainBoth();

    readAllBroadcast();

    const uint32_t t0 = millis();
    const uint32_t windowMs = 30;

    while ((millis() - t0) < windowMs) {
        CAN_message_t rx;
        while (Can1.read(rx)) {
            uint8_t mid = 0;
            if (parseReadAllReply(rx, mid))
                addFound(mid, 1);
        }
        while (Can2.read(rx)) {
            uint8_t mid = 0;
            if (parseReadAllReply(rx, mid))
                addFound(mid, 2);
        }
    }
    drainBoth();
}

static void writeCanIdFrame(uint8_t motorId, uint8_t bus, bool readFlag, uint8_t canIdParam)
{
    CAN_message_t msg{};
    msg.id = motorCmdId(motorId);
    msg.len = 8;
    memset(msg.buf, 0, 8);
    msg.buf[0] = CMD_CAN_ID;
    msg.buf[1] = 0x00;
    msg.buf[2] = readFlag ? (uint8_t)1 : (uint8_t)0;
    Serial.print("R/W: ");
    Serial.println(msg.buf[2]);
    msg.buf[7] = canIdParam;

    if (bus == 2)
        Can2.write(msg);
    else
        Can1.write(msg);
}

// Read CAN ID (0x79, DATA[2]=1). Returns true if a matching reply was seen.
static bool readCanIdRegister(uint8_t motorId, uint8_t bus, uint16_t &canIdOut)
{
    drainBoth();
    writeCanIdFrame(motorId, bus, true, 0);

    const uint32_t t0 = millis();
    while ((millis() - t0) < 50) {
        CAN_message_t rx;
        while (bus == 2 ? Can2.read(rx) : Can1.read(rx)) {

            if (rx.id != motorReplyId(motorId))
                continue;

            // 🔴 CRITICAL: ignore status frames
            if (rx.buf[0] != CMD_CAN_ID)
                continue;

            if (rx.len < 8)
                continue;

            uint16_t raw = (uint16_t)rx.buf[6] | ((uint16_t)rx.buf[7] << 8);

            Serial.print("Raw CAN ID: 0x");
            Serial.println(raw, HEX);

            uint8_t decoded = raw - 0x240;

            Serial.print("Decoded motor ID: ");
            Serial.println(decoded);

            canIdOut = raw;
            return true;
        }
    }
    drainBoth();
    return false;
}

// Write new CAN ID (1..32). Expect echo of command on reply ID per manual.
static bool writeCanIdRegister(uint8_t motorId, uint8_t bus, uint8_t newCanId1to32)
{
    if (newCanId1to32 < 1 || newCanId1to32 > 32)
        return false;

    drainBoth();
    writeCanIdFrame(motorId, bus, false, newCanId1to32);

    const uint32_t t0 = millis();
    bool ok = false;
    while ((millis() - t0) < 80) {
        CAN_message_t rx;
        while (bus == 2 ? Can2.read(rx) : Can1.read(rx)) {
            if (rx.id != motorReplyId(motorId))
                continue;
            if (rx.len < 8)
                continue;
            if (rx.buf[0] != CMD_CAN_ID)
                continue;
            if (rx.buf[2] != 0)
                continue;
            if (rx.buf[7] == newCanId1to32)
                ok = true;
        }
        if (ok)
            break;
    }
    drainBoth();
    return ok;
}

static void printFoundList()
{
    Serial.println();
    if (g_nFound == 0) {
        Serial.println("No motors found. Check power, termination, and run 's' to scan again.");
        return;
    }
    Serial.println("Discovered motors (from readAll / broadcast 0x9C):");
    for (uint8_t i = 0; i < g_nFound; i++) {
        Serial.print("  [");
        Serial.print(i);
        Serial.print("]  CAN ID (logical) ");
        Serial.print(g_found[i].motorId);
        Serial.print("  -> TX 0x");
        Serial.print(motorCmdId(g_found[i].motorId), HEX);
        Serial.print("  bus CAN");
        Serial.println(g_found[i].bus);
    }
    Serial.println("Enter list index [0..n-1], then NEW motor ID (1..32) on separate lines.");
}

static void promptMenu()
{
    Serial.println();
    Serial.println("=== CAN ID tool ===");
    Serial.println("  s = scan (readAll)");
    Serial.println("  r <motorId 1-32> <bus 1|2> = read CAN ID via 0x79");
    Serial.println("  c = change ID (interactive, uses last scan)");
    Serial.println("===================");
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }

    Can1.begin();
    Can1.setBaudRate(1000000);
    Can2.begin();
    Can2.setBaudRate(1000000);

    Serial.println("Teensy CAN ID reassignment ready.");
    scanMotors();
    printFoundList();
    promptMenu();
}

void loop()
{
    if (!Serial.available())
        return;

    char c = (char)Serial.peek();
    if (c == 's' || c == 'S') {
        Serial.read();
        while (Serial.available())
            Serial.read();
        Serial.println("Scanning...");
        scanMotors();
        printFoundList();
        promptMenu();
        return;
    }

    if (c == 'r' || c == 'R') {
        Serial.read();
        int mid = Serial.parseInt();
        int bus = Serial.parseInt();
        while (Serial.available() && (Serial.peek() == '\r' || Serial.peek() == '\n'))
            Serial.read();
        if (mid < 1 || mid > 32 || (bus != 1 && bus != 2)) {
            Serial.println("Usage: r <motorId 1-32> <bus 1 or 2>");
            return;
        }
        uint16_t cid = 0;
        if (readCanIdRegister((uint8_t)mid, (uint8_t)bus, cid)) {
            Serial.print("Read CAN ID register: ");
            Serial.print(cid);
            Serial.print(" (expect logical 1..32, raw=");
            Serial.print(cid);
            Serial.println(")");
        } else {
            Serial.println("Read failed (no 0x79 reply).");
        }
        promptMenu();
        return;
    }

    if (c == 'c' || c == 'C') {
        Serial.read();
        while (Serial.available())
            Serial.read();

        if (g_nFound == 0) {
            Serial.println("Scan first ('s'). No motors in list.");
            promptMenu();
            return;
        }
        printFoundList();

        Serial.println("List index to reassign:");
        while (!Serial.available()) {
        }
        int idx = Serial.parseInt();
        while (Serial.available() && (Serial.peek() == '\r' || Serial.peek() == '\n' || Serial.peek() == ' '))
            Serial.read();

        if (idx < 0 || idx >= g_nFound) {
            Serial.println("Invalid index.");
            promptMenu();
            return;
        }

        Serial.println("New CAN / motor ID (1..32):");
        while (!Serial.available()) {
        }
        int newId = Serial.parseInt();
        while (Serial.available())
            Serial.read();

        if (newId < 1 || newId > 32) {
            Serial.println("New ID must be 1..32.");
            promptMenu();
            return;
        }

        uint8_t oldId = g_found[idx].motorId;
        uint8_t bus = g_found[idx].bus;

        Serial.print("Writing: motor ");
        Serial.print(oldId);
        Serial.print(" on CAN");
        Serial.print(bus);
        Serial.print(" -> new ID ");
        Serial.println(newId);

        if (writeCanIdRegister(oldId, bus, (uint8_t)newId)) {
            Serial.println("SUCCESS: motor acknowledged CAN ID write (0x79 echo).");
            Serial.println("Power-cycle or re-scan ('s') to confirm on the bus.");
        } else {
            Serial.println("FAILED: no matching 0x79 write echo. Check wiring, bus, and motor state.");
        }
        promptMenu();
        return;
    }

    // Unknown char: consume one
    Serial.read();
}
