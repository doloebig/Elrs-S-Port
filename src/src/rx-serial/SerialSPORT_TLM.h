#pragma once

#if defined(TARGET_RX)

#include "SerialIO.h"
#include "device.h"

//
// FrSky S.Port telemetry input (ExpressLRS receiver acts as S.Port bus master/poller).
//
// The receiver polls the S.Port bus round-robin (0x7E + physical ID), reads the
// sensor data frames, decodes the FrSky "application IDs" and forwards the values
// as CRSF telemetry to the handset - exactly the same idea as the HoTT telemetry
// driver (SerialHoTT_TLM), just for the FrSky Smart Port sensor family.
//
// Wire level: single wire half duplex, 57600 baud, 8N1, signal INVERTED.
// (The UART inversion is configured in rx_main.cpp / setupSerial(); on ESP32 the
// half duplex direction switching is done here via the GPIO matrix, matching HoTT.)
//

#ifndef PACKED
#define PACKED __attribute__((packed))
#endif

#define SPORT_MAX_BUF_LEN 64      // max buffer size for serial in data

// S.Port bus framing
#define SPORT_POLL_HEADER 0x7E    // start-of-poll byte the master sends
#define SPORT_DATA_FRAME  0x10    // sensor -> master "data frame" type byte
#define SPORT_BYTESTUFF   0x7D    // byte-stuffing escape marker
#define SPORT_STUFF_MASK  0x20    // XOR mask applied to the stuffed byte
#define SPORT_FRAME_LEN   8       // type(1) + appId(2) + value(4) + crc(1)

// S.Port poll timing [ms]
#define SPORT_POLL_RATE       12  // gap between two consecutive device polls
#define SPORT_CMD_DELAY        1  // delay before releasing the bus after the poll
#define SPORT_WAIT_TX_COMPLETE 2  // wait for the poll bytes to be shifted out
#define SPORT_RX_TIMEOUT      10  // give up waiting for a sensor reply after this

// number of physical IDs on the bus (FrSky standard poll table)
#define SPORT_PHYSICAL_ID_COUNT 28

// Diagnostic telemetry: when 1, the driver reports what it is decoding to the
// handset as RPM values (plain, unclamped numbers on the radio - unlike
// temperatures, which Ethos limits to -30..+100 C):
//   RPM (rpm0) = appID of the last received frame (decimal, e.g. 512 = 0x0200
//                current, 528 = 0x0210 VFAS voltage) - cycles through whatever
//                the sensor actually sends
//   rpm1       = last decoded voltage  (0.1 V, e.g. 250 = 25.0 V)
//   rpm2       = last decoded current  (0.1 A, e.g. 15 = 1.5 A)
// Set this to 0 once telemetry works, then rebuild.
#define SPORT_DEBUG_TELEM 1

// Device discovery: for the first SPORT_DISCOVERY_TIMEOUT [ms] after the link
// comes up all 28 physical IDs are polled to find out which ones actually have
// a sensor. Afterwards only the responding IDs are polled, which shortens the
// poll cycle and raises the telemetry update rate.
#define SPORT_DISCOVERY_TIMEOUT 30000

// CRSF send scheduling: a CRSF telemetry packet is (re)sent when its min-rate
// timer [ms] has expired or when the packet content has changed.
#define SPORT_BATT_MIN_CRSFRATE     5000
#define SPORT_VARIO_MIN_CRSFRATE    1000
#define SPORT_GPS_MIN_CRSFRATE      5000
#define SPORT_RPM_MIN_CRSFRATE      5000
#define SPORT_TEMP_MIN_CRSFRATE     5000
#define SPORT_CELLS_MIN_CRSFRATE    5000
#define SPORT_AIRSPEED_MIN_CRSFRATE 5000

// how long a decoded value is considered "present" without a refresh [ms]
#define SPORT_VALUE_TIMEOUT 5000

//
// FrSky S.Port application ID ranges (first ID of each 16-slot block).
// A concrete sensor uses FIRST_ID + instance, so we mask the low nibble away
// when classifying an incoming frame.
//
#define SPORT_ID_MASK              0xFFF0
#define SPORT_ALT_FIRST_ID         0x0100  // barometric altitude, cm  (int32)
#define SPORT_VARIO_FIRST_ID       0x0110  // vertical speed, cm/s     (int32)
#define SPORT_CURR_FIRST_ID        0x0200  // current, 0.1 A
#define SPORT_VFAS_FIRST_ID        0x0210  // voltage, 0.01 V
#define SPORT_CELLS_FIRST_ID       0x0300  // battery cells (packed, 2 cells/frame)
#define SPORT_T1_FIRST_ID          0x0400  // temperature 1, deg C
#define SPORT_T2_FIRST_ID          0x0410  // temperature 2, deg C
#define SPORT_RPM_FIRST_ID         0x0500  // rpm
#define SPORT_FUEL_FIRST_ID        0x0600  // fuel level: 0..100 %, or consumed mAh

// A FUEL value at or below this is treated as a percentage, above it as mAh.
// (FrSky uses the same appId 0x0600 for both a 0..100% gauge and a consumed
//  capacity in mAh; the magnitude is the only thing that tells them apart.)
#define SPORT_FUEL_PERCENT_MAX 100
#define SPORT_GPS_LONG_LATI_FIRST_ID 0x0800 // GPS lat/lon (packed)
#define SPORT_GPS_ALT_FIRST_ID     0x0820  // GPS altitude, cm         (int32)
#define SPORT_GPS_SPEED_FIRST_ID   0x0830  // GPS ground speed, knots * 1000
#define SPORT_GPS_COURS_FIRST_ID   0x0840  // GPS course, deg * 100
#define SPORT_AIR_SPEED_FIRST_ID   0x0A00  // air speed, knots * 10

//
// A raw sensor data frame as it is reassembled from the (de-stuffed) bus bytes.
//
typedef struct
{
    uint8_t frameType;   // 0 SPORT_DATA_FRAME (0x10)
    uint16_t appId;      // 1 application/data ID (little endian on the wire)
    uint32_t value;      // 3 payload (little endian on the wire)
    uint8_t crc;         // 7 frame CRC
} PACKED sportFrame_t;

//
// Latest decoded S.Port telemetry values, already normalised to sensible units.
// Every field carries a "last seen" timestamp so we only forward telemetry for
// values that a sensor on the bus actually provides.
//
typedef struct
{
    // battery
    uint16_t voltage;        // 0.1 V   (CRSF battery voltage unit)
    uint16_t current;        // 0.1 A
    uint16_t fuelPercent;    // %        -> CRSF battery "remaining"
    uint32_t capacityMah;    // mAh      -> CRSF battery "capacity"
    bool haveCapacity;
    uint32_t lastBatt;

    // cells
    uint16_t cellVoltage[16]; // mV per cell
    uint8_t  cellCount;
    uint32_t lastCells;

    // vario / baro
    int32_t altitudeCm;      // cm
    int32_t vspeedCmS;       // cm/s
    uint32_t lastVario;

    // gps
    int32_t latitude;        // degree * 1e7
    int32_t longitude;       // degree * 1e7
    uint16_t groundspeed;    // 0.1 km/h (CRSF unit)
    uint16_t heading;        // degree * 100
    int32_t gpsAltitudeCm;   // cm
    uint32_t lastGps;
    bool haveLat;
    bool haveLon;

    // rpm
    int32_t rpm;
    uint32_t lastRpm;

    // temperatures
    int16_t temp1;           // deg C
    int16_t temp2;           // deg C
    bool haveTemp1;
    bool haveTemp2;
    uint32_t lastTemp;

    // airspeed
    uint16_t airspeed;       // 0.1 km/h (CRSF unit)
    uint32_t lastAirspeed;
} sportTelemetry_t;

class SerialSPORT_TLM final : public SerialIO
{
public:
    SerialSPORT_TLM(Stream &out, Stream &in, int8_t serial1TXpin = UNDEF_PIN);
    ~SerialSPORT_TLM() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override { return DURATION_IMMEDIATELY; };

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

private:
#if defined(PLATFORM_ESP32)
    int8_t halfDuplexPin;
    uint8_t UTXDoutIdx;
    uint8_t URXDinIdx;
#endif

    void setTXMode();
    void setRXMode();

    void processBytes(uint8_t *bytes, uint16_t size) override;
    void processFrame(const sportFrame_t *frame);
    static uint8_t calcFrameCRC(const uint8_t *buf);

    void scheduleDevicePolling(uint32_t now);

    void scheduleCRSFtelemetry(uint32_t now);
#if SPORT_DEBUG_TELEM
    void sendCRSFdebug(uint32_t now);
#endif
    void sendCRSFvario(uint32_t now);
    void sendCRSFgps(uint32_t now);
    void sendCRSFbattery(uint32_t now);
    void sendCRSFrpm(uint32_t now);
    void sendCRSFtemp(uint32_t now);
    void sendCRSFcells(uint32_t now);
    void sendCRSFairspeed(uint32_t now);

    static inline bool fresh(uint32_t now, uint32_t last)
    {
        return last != 0 && (now - last) < SPORT_VALUE_TIMEOUT;
    }

    // decoded telemetry store
    sportTelemetry_t tlm = {0};

    // input byte-stream de-stuffing state
    uint8_t frameBuffer[SPORT_FRAME_LEN];
    uint8_t frameIndex = 0;
    bool escaping = false;

    // poll scheduler / device discovery state
    uint32_t lastPoll = 0;
    uint8_t nextPhysical = 0;         // index of the next ID to consider polling
    uint8_t currentPollIndex = 0;     // index of the ID whose reply we're awaiting
    bool discoveryMode = true;
    uint32_t discoveryTimerStart = 0;
    bool physicalPresent[SPORT_PHYSICAL_ID_COUNT] = {false};

    // consumed-capacity (mAh) tracking
    bool haveSensorCapacity = false;  // true once a sensor reports mAh directly
    uint32_t lastCurrentMillis = 0;   // for coulomb counting from the current sensor
    float consumedMah = 0.0f;

    void markPhysicalPresent();
    bool anyPhysicalPresent() const;

#if SPORT_DEBUG_TELEM
    // diagnostic state (see SPORT_DEBUG_TELEM)
    uint32_t dbgPollCount = 0;
    uint32_t dbgRxByteCount = 0;
    uint32_t dbgFrameCount = 0;
    uint16_t dbgLastAppId = 0;
    uint32_t lastDebugSent = 0;
#endif

    // CRSF send scheduler state
    uint32_t lastVarioSent = 0;
    uint32_t lastGPSSent = 0;
    uint32_t lastBatterySent = 0;
    uint32_t lastRpmSent = 0;
    uint32_t lastTempSent = 0;
    uint32_t lastCellsSent = 0;
    uint32_t lastAirspeedSent = 0;

    uint8_t lastVarioCRC = 0;
    uint8_t lastGPSCRC = 0;
    uint8_t lastBatteryCRC = 0;
    uint8_t lastRpmCRC = 0;
    uint8_t lastTempCRC = 0;
    uint8_t lastCellsCRC = 0;
    uint8_t lastAirspeedCRC = 0;
};

#endif
