#if defined(TARGET_RX)

#include "SerialSPORT_TLM.h"

#include "CRSFRouter.h"
#include "common.h"

constexpr uint8_t SIZE_8BIT = 1;
constexpr uint8_t SIZE_16BIT = 2;
constexpr uint8_t SIZE_24BIT = 3;

//
// FrSky S.Port physical ID poll table (5-bit ID + 3-bit CRC in the upper bits).
// These are the 28 addresses the bus master polls round-robin.
//
static const uint8_t sportPhysicalIDs[SPORT_PHYSICAL_ID_COUNT] = {
    0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67,
    0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0x0D, 0x8E, 0x2F,
    0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7,
    0x98, 0x39, 0xBA, 0x1B};

SerialSPORT_TLM::SerialSPORT_TLM(Stream &out, Stream &in, const int8_t serial1TXpin)
    : SerialIO(&out, &in)
{
#if defined(PLATFORM_ESP32)
    if (serial1TXpin == UNDEF_PIN)
    {
        // UART0: use default TX pin for half duplex if not defined otherwise
        UTXDoutIdx = U0TXD_OUT_IDX;
        URXDinIdx = U0RXD_IN_IDX;
        halfDuplexPin = GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN ? U0TXD_GPIO_NUM : GPIO_PIN_RCSIGNAL_TX;
    }
    else
    {
        // UART1: use Serial1 TX assigned pin for half duplex
        UTXDoutIdx = U1TXD_OUT_IDX;
        URXDinIdx = U1RXD_IN_IDX;
        halfDuplexPin = serial1TXpin;
    }
#endif

    uint32_t now = millis();
    lastPoll = now;
    discoveryTimerStart = now;
}

int SerialSPORT_TLM::getMaxSerialReadSize()
{
    return SPORT_MAX_BUF_LEN;
}

// mark the physical ID we last polled as "present" (a valid reply came back)
void SerialSPORT_TLM::markPhysicalPresent()
{
    if (currentPollIndex < SPORT_PHYSICAL_ID_COUNT)
    {
        physicalPresent[currentPollIndex] = true;
    }
}

bool SerialSPORT_TLM::anyPhysicalPresent() const
{
    for (uint8_t i = 0; i < SPORT_PHYSICAL_ID_COUNT; i++)
    {
        if (physicalPresent[i])
        {
            return true;
        }
    }
    return false;
}

void SerialSPORT_TLM::setTXMode()
{
#if defined(PLATFORM_ESP32)
    // S.Port is an inverted single wire bus. The UART inversion itself is set up
    // in setupSerial()/setupSerial1(); here we only switch the shared half-duplex
    // GPIO between the UART TX and RX signals, exactly like the HoTT driver.
    pinMode(halfDuplexPin, OUTPUT);
    digitalWrite(halfDuplexPin, HIGH);
    pinMatrixOutAttach(halfDuplexPin, UTXDoutIdx, false, false);
#endif
}

void SerialSPORT_TLM::setRXMode()
{
#if defined(PLATFORM_ESP32)
    pinMode(halfDuplexPin, INPUT_PULLUP);
    pinMatrixInAttach(halfDuplexPin, URXDinIdx, false);
#endif
}

//
// S.Port frame CRC: sum of frameType + appId + value (7 bytes), fold the carry
// back in, then take the one's complement. The received CRC is the 8th byte.
//
uint8_t SerialSPORT_TLM::calcFrameCRC(const uint8_t *buf)
{
    uint16_t crc = 0;

    for (uint8_t i = 0; i < SPORT_FRAME_LEN - 1; i++)
    {
        crc += buf[i];
        crc += crc >> 8;
        crc &= 0x00FF;
    }

    return (uint8_t)(0xFF - crc);
}

//
// De-stuff the incoming bus bytes and reassemble 8-byte sensor data frames.
// 0x7D is the escape marker (next byte ^ 0x20); 0x7E is a poll header used to
// re-align the assembler. A frame always starts with the data-frame type 0x10.
//
void SerialSPORT_TLM::processBytes(uint8_t *bytes, uint16_t size)
{
#if SPORT_DEBUG_TELEM
    dbgRxByteCount += size;
#endif

    for (uint16_t i = 0; i < size; i++)
    {
        uint8_t b = bytes[i];

        if (b == SPORT_POLL_HEADER)
        {
            // start of a (new) poll cycle - drop whatever partial frame we had
            frameIndex = 0;
            escaping = false;
            continue;
        }

        if (b == SPORT_BYTESTUFF)
        {
            escaping = true;
            continue;
        }

        if (escaping)
        {
            b ^= SPORT_STUFF_MASK;
            escaping = false;
        }

        // wait for a valid frame start (data-frame type byte)
        if (frameIndex == 0 && b != SPORT_DATA_FRAME)
        {
            continue;
        }

        frameBuffer[frameIndex++] = b;

        if (frameIndex == SPORT_FRAME_LEN)
        {
            frameIndex = 0;

            if (frameBuffer[0] == SPORT_DATA_FRAME &&
                frameBuffer[SPORT_FRAME_LEN - 1] == calcFrameCRC(frameBuffer))
            {
                // a valid reply -> the physical ID we just polled has a sensor
                markPhysicalPresent();
#if SPORT_DEBUG_TELEM
                dbgFrameCount++;
#endif

                sportFrame_t f;
                f.frameType = frameBuffer[0];
                f.appId = (uint16_t)frameBuffer[1] | ((uint16_t)frameBuffer[2] << 8);
                f.value = (uint32_t)frameBuffer[3] |
                          ((uint32_t)frameBuffer[4] << 8) |
                          ((uint32_t)frameBuffer[5] << 16) |
                          ((uint32_t)frameBuffer[6] << 24);
                f.crc = frameBuffer[SPORT_FRAME_LEN - 1];

                processFrame(&f);
            }
        }
    }
}

//
// Decode one received S.Port data frame into the normalised telemetry store.
//
void SerialSPORT_TLM::processFrame(const sportFrame_t *frame)
{
    const uint32_t now = millis();
    const uint16_t id = frame->appId & SPORT_ID_MASK;
    const uint32_t raw = frame->value;

#if SPORT_DEBUG_TELEM
    dbgLastAppId = frame->appId; // report the raw appID for diagnostics
#endif

    switch (id)
    {
    case SPORT_VFAS_FIRST_ID: // voltage, 0.01 V -> CRSF 0.1 V
        tlm.voltage = (uint16_t)(raw / 10);
        tlm.lastBatt = now;
        break;

    case SPORT_CURR_FIRST_ID: // current, 0.1 A (== CRSF unit)
        tlm.current = (uint16_t)raw;
        // Coulomb-count the current into a consumed-capacity estimate so we can
        // report mAh even when no sensor sends the capacity directly. Only used
        // as long as no sensor reports mAh itself (see FUEL handling below).
        if (lastCurrentMillis != 0)
        {
            uint32_t dt = now - lastCurrentMillis;                 // ms
            consumedMah += ((float)tlm.current * (float)dt) / 36000.0f; // 0.1A * ms -> mAh
            if (!haveSensorCapacity)
            {
                tlm.capacityMah = (uint32_t)consumedMah;
                tlm.haveCapacity = true;
            }
        }
        lastCurrentMillis = now;
        tlm.lastBatt = now;
        break;

    case SPORT_FUEL_FIRST_ID:
        // FrSky reuses appId 0x0600 for a 0..100 % gauge and for a consumed
        // capacity in mAh. Small values are treated as a percentage (-> CRSF
        // "remaining"), larger ones as mAh (-> CRSF "capacity", authoritative).
        if (raw > SPORT_FUEL_PERCENT_MAX)
        {
            tlm.capacityMah = raw;
            tlm.haveCapacity = true;
            haveSensorCapacity = true; // sensor provides mAh, stop coulomb counting
        }
        else
        {
            tlm.fuelPercent = (uint16_t)raw;
        }
        tlm.lastBatt = now;
        break;

    case SPORT_CELLS_FIRST_ID: // two packed cell voltages per frame
    {
        uint8_t total = raw & 0x0F;
        uint8_t idx = (raw >> 4) & 0x0F;
        uint16_t v1 = (raw >> 8) & 0x0FFF;  // unit 0.002 V
        uint16_t v2 = (raw >> 20) & 0x0FFF; // unit 0.002 V

        tlm.cellCount = total > 16 ? 16 : total;

        if (idx < 16)
        {
            tlm.cellVoltage[idx] = v1 * 2; // -> mV
        }
        if ((uint8_t)(idx + 1) < 16 && (uint8_t)(idx + 1) < total)
        {
            tlm.cellVoltage[idx + 1] = v2 * 2; // -> mV
        }
        tlm.lastCells = now;
        break;
    }

    case SPORT_ALT_FIRST_ID: // barometric altitude, cm
        tlm.altitudeCm = (int32_t)raw;
        tlm.lastVario = now;
        break;

    case SPORT_VARIO_FIRST_ID: // vertical speed, cm/s
        tlm.vspeedCmS = (int32_t)raw;
        tlm.lastVario = now;
        break;

    case SPORT_GPS_LONG_LATI_FIRST_ID: // packed lat/lon
    {
        int32_t coord = (int32_t)(raw & 0x3FFFFFFF); // minutes * 1e4
        int32_t degE7 = (int32_t)(((int64_t)coord * 100) / 6);

        if (raw & 0x40000000) // sign bit: S / W
        {
            degE7 = -degE7;
        }

        if (raw & 0x80000000) // longitude
        {
            tlm.longitude = degE7;
            tlm.haveLon = true;
        }
        else // latitude
        {
            tlm.latitude = degE7;
            tlm.haveLat = true;
        }
        tlm.lastGps = now;
        break;
    }

    case SPORT_GPS_ALT_FIRST_ID: // GPS altitude, cm
        tlm.gpsAltitudeCm = (int32_t)raw;
        tlm.lastGps = now;
        break;

    case SPORT_GPS_SPEED_FIRST_ID: // knots * 1000 -> CRSF 0.1 km/h
        tlm.groundspeed = (uint16_t)(((uint64_t)raw * 1852) / 100000);
        tlm.lastGps = now;
        break;

    case SPORT_GPS_COURS_FIRST_ID: // course, deg * 100 (== CRSF unit)
        tlm.heading = (uint16_t)raw;
        tlm.lastGps = now;
        break;

    case SPORT_T1_FIRST_ID: // temperature 1, deg C
        tlm.temp1 = (int16_t)(int32_t)raw;
        tlm.haveTemp1 = true;
        tlm.lastTemp = now;
        break;

    case SPORT_T2_FIRST_ID: // temperature 2, deg C
        tlm.temp2 = (int16_t)(int32_t)raw;
        tlm.haveTemp2 = true;
        tlm.lastTemp = now;
        break;

    case SPORT_RPM_FIRST_ID: // rpm
        tlm.rpm = (int32_t)raw;
        tlm.lastRpm = now;
        break;

    case SPORT_AIR_SPEED_FIRST_ID: // knots * 10 -> CRSF 0.1 km/h
        tlm.airspeed = (uint16_t)(((uint64_t)raw * 1852) / 1000);
        tlm.lastAirspeed = now;
        break;

    default:
        break;
    }
}

void SerialSPORT_TLM::sendQueuedData(uint32_t maxBytesToSend)
{
    uint32_t now = millis();

    // keep the discovery window from running while we are not connected
    if (connectionState != connected)
    {
        discoveryTimerStart = now;
    }

    // leave discovery mode once the window has elapsed AND we have found at least
    // one sensor; otherwise keep scanning so a late/added sensor is still found.
    if (discoveryMode && (now - discoveryTimerStart >= SPORT_DISCOVERY_TIMEOUT))
    {
        if (anyPhysicalPresent())
        {
            discoveryMode = false;
        }
        else
        {
            discoveryTimerStart = now; // re-arm, nothing discovered yet
        }
    }

    // poll the S.Port bus
    scheduleDevicePolling(now);

    // forward decoded telemetry as CRSF
    scheduleCRSFtelemetry(now);
}

void SerialSPORT_TLM::scheduleDevicePolling(uint32_t now)
{
    // time to poll the next physical ID
    if (now - lastPoll >= SPORT_POLL_RATE)
    {
        lastPoll = now;

        // Work out which physical ID to poll next: all of them while in
        // discovery mode, only the ones that have answered once we are past it.
        uint8_t pollIndex = SPORT_PHYSICAL_ID_COUNT;
        for (uint8_t i = 0; i < SPORT_PHYSICAL_ID_COUNT; i++)
        {
            uint8_t idx = (nextPhysical + i) % SPORT_PHYSICAL_ID_COUNT;
            if (discoveryMode || physicalPresent[idx])
            {
                pollIndex = idx;
                nextPhysical = (idx + 1) % SPORT_PHYSICAL_ID_COUNT;
                break;
            }
        }

        // nothing to poll (should not happen: discovery re-arms if empty)
        if (pollIndex == SPORT_PHYSICAL_ID_COUNT)
        {
            return;
        }

        currentPollIndex = pollIndex;
        uint8_t physId = sportPhysicalIDs[pollIndex];

        // re-align the frame assembler for the upcoming reply
        frameIndex = 0;
        escaping = false;

#if SPORT_DEBUG_TELEM
        dbgPollCount++;
#endif

        // switch to half duplex TX, send the 2-byte poll (0x7E + physical ID),
        // wait for both bytes to be shifted out, then release the bus to RX
        // immediately so we don't miss a fast sensor's reply.
        setTXMode();
        _outputPort->write(SPORT_POLL_HEADER);
        _outputPort->write(physId);
        _outputPort->flush();
        setRXMode();
    }
}

void SerialSPORT_TLM::scheduleCRSFtelemetry(uint32_t now)
{
#if SPORT_DEBUG_TELEM
    sendCRSFdebug(now);
#endif
    sendCRSFvario(now);
    sendCRSFgps(now);
    sendCRSFbattery(now);
    sendCRSFcells(now);
    sendCRSFrpm(now);
    sendCRSFtemp(now);
    sendCRSFairspeed(now);
}

#if SPORT_DEBUG_TELEM
void SerialSPORT_TLM::sendCRSFdebug(uint32_t now)
{
    // ~4 Hz heartbeat reporting what we decode, as an RPM frame. RPM shows as a
    // plain, unclamped number on the handset. Sending faster than 1 Hz keeps the
    // radio from announcing "sensor lost" when the odd telemetry packet drops.
    if (now - lastDebugSent < 250)
    {
        return;
    }
    lastDebugSent = now;

    CRSF_MK_FRAME_T(crsf_sensor_rpm_t) crsfDbg = {0};
    crsfDbg.p.source_id = 99; // arbitrary, keeps it distinct from real RPM sensors
    crsfDbg.p.rpm0 = htobe24(dbgLastAppId); // RPM : appID of last frame (512=curr, 528=vfas, ...)
    crsfDbg.p.rpm1 = htobe24(tlm.voltage);  // rpm1: decoded voltage (0.1 V)
    crsfDbg.p.rpm2 = htobe24(tlm.current);  // rpm2: decoded current (0.1 A)

    uint8_t payloadSize = 1 + 3 * 3; // source_id + three 24-bit values
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfDbg, CRSF_FRAMETYPE_RPM, CRSF_FRAME_SIZE(payloadSize));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfDbg.h);
}
#endif

void SerialSPORT_TLM::sendCRSFvario(uint32_t now)
{
    if (!fresh(now, tlm.lastVario))
    {
        return;
    }

    // indicate external baro/vario sensor is present
    crsfBaroSensorDetected = true;

    CRSF_MK_FRAME_T(crsf_sensor_baro_vario_t) crsfBaro = {0};
    crsfBaro.p.altitude = htobe16((uint16_t)(tlm.altitudeCm / 10 + 10000)); // cm -> dm, +10000 offset
    crsfBaro.p.verticalspd = htobe16((int16_t)tlm.vspeedCmS);
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfBaro, CRSF_FRAMETYPE_BARO_ALTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_baro_vario_t)));

    if ((now - lastVarioSent >= SPORT_VARIO_MIN_CRSFRATE) || (lastVarioCRC != crsfBaro.crc))
    {
        lastVarioSent = now;
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfBaro.h);
    }

    lastVarioCRC = crsfBaro.crc;
}

void SerialSPORT_TLM::sendCRSFgps(uint32_t now)
{
    if (!fresh(now, tlm.lastGps) || !(tlm.haveLat || tlm.haveLon))
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_gps_t) crsfGPS = {0};
    crsfGPS.p.latitude = htobe32(tlm.latitude);
    crsfGPS.p.longitude = htobe32(tlm.longitude);
    crsfGPS.p.groundspeed = htobe16(tlm.groundspeed);
    crsfGPS.p.gps_heading = htobe16(tlm.heading);
    crsfGPS.p.altitude = htobe16((uint16_t)(tlm.gpsAltitudeCm / 100 + 1000)); // cm -> m, +1000 offset
    crsfGPS.p.satellites_in_use = 0;
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfGPS, CRSF_FRAMETYPE_GPS, CRSF_FRAME_SIZE(sizeof(crsf_sensor_gps_t)));

    if ((now - lastGPSSent >= SPORT_GPS_MIN_CRSFRATE) || (lastGPSCRC != crsfGPS.crc))
    {
        lastGPSSent = now;
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfGPS.h);
    }

    lastGPSCRC = crsfGPS.crc;
}

void SerialSPORT_TLM::sendCRSFbattery(uint32_t now)
{
    if (!fresh(now, tlm.lastBatt))
    {
        return;
    }

    // indicate external battery sensor is present
    crsfBatterySensorDetected = true;

    CRSF_MK_FRAME_T(crsf_sensor_battery_t) crsfBatt = {0};
    crsfBatt.p.voltage = htobe16(tlm.voltage);
    crsfBatt.p.current = htobe16(tlm.current);
    crsfBatt.p.capacity = htobe24(tlm.haveCapacity ? tlm.capacityMah : 0); // consumed mAh
    crsfBatt.p.remaining = (uint8_t)tlm.fuelPercent;
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfBatt, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)));

    if ((now - lastBatterySent >= SPORT_BATT_MIN_CRSFRATE) || (lastBatteryCRC != crsfBatt.crc))
    {
        lastBatterySent = now;
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfBatt.h);
    }

    lastBatteryCRC = crsfBatt.crc;
}

void SerialSPORT_TLM::sendCRSFcells(uint32_t now)
{
    if (!fresh(now, tlm.lastCells) || tlm.cellCount == 0)
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_cells_t) crsfCells = {0};
    crsfCells.p.source_id = 0;

    uint8_t n = tlm.cellCount > 16 ? 16 : tlm.cellCount;
    for (uint8_t i = 0; i < n; i++)
    {
        crsfCells.p.cell[i] = htobe16(tlm.cellVoltage[i]);
    }

    uint8_t payloadSize = SIZE_8BIT + SIZE_16BIT * n;
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfCells, CRSF_FRAMETYPE_CELLS, CRSF_FRAME_SIZE(payloadSize));

    uint8_t crc = ((uint8_t *)&crsfCells.p)[payloadSize];

    if ((now - lastCellsSent >= SPORT_CELLS_MIN_CRSFRATE) || (lastCellsCRC != crc))
    {
        lastCellsSent = now;
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfCells.h);
    }

    lastCellsCRC = crc;
}

void SerialSPORT_TLM::sendCRSFrpm(uint32_t now)
{
    if (!fresh(now, tlm.lastRpm))
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_rpm_t) crsfRpm = {0};
    crsfRpm.p.source_id = 0;
    crsfRpm.p.rpm0 = htobe24((uint32_t)tlm.rpm);

    uint8_t payloadSize = SIZE_8BIT + SIZE_24BIT * 1;
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfRpm, CRSF_FRAMETYPE_RPM, CRSF_FRAME_SIZE(payloadSize));

    uint8_t crc = ((uint8_t *)&crsfRpm.p)[payloadSize];

    if ((now - lastRpmSent >= SPORT_RPM_MIN_CRSFRATE) || (lastRpmCRC != crc))
    {
        lastRpmSent = now;
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfRpm.h);
    }

    lastRpmCRC = crc;
}

void SerialSPORT_TLM::sendCRSFtemp(uint32_t now)
{
    if (!fresh(now, tlm.lastTemp) || !(tlm.haveTemp1 || tlm.haveTemp2))
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_temp_t) crsfTemp = {0};
    crsfTemp.p.source_id = 0;

    uint8_t count = 0;
    if (tlm.haveTemp1)
    {
        crsfTemp.p.temperature[count++] = htobe16((int16_t)(tlm.temp1 * 10)); // -> deci-degree
    }
    if (tlm.haveTemp2)
    {
        crsfTemp.p.temperature[count++] = htobe16((int16_t)(tlm.temp2 * 10));
    }

    uint8_t payloadSize = SIZE_8BIT + SIZE_16BIT * count;
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfTemp, CRSF_FRAMETYPE_TEMP, CRSF_FRAME_SIZE(payloadSize));

    uint8_t crc = ((uint8_t *)&crsfTemp.p)[payloadSize];

    if ((now - lastTempSent >= SPORT_TEMP_MIN_CRSFRATE) || (lastTempCRC != crc))
    {
        lastTempSent = now;
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfTemp.h);
    }

    lastTempCRC = crc;
}

void SerialSPORT_TLM::sendCRSFairspeed(uint32_t now)
{
    if (!fresh(now, tlm.lastAirspeed))
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_airspeed_t) crsfAirspeed = {0};
    crsfAirspeed.p.speed = htobe16(tlm.airspeed);
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfAirspeed, CRSF_FRAMETYPE_AIRSPEED, CRSF_FRAME_SIZE(sizeof(crsf_sensor_airspeed_t)));

    if ((now - lastAirspeedSent >= SPORT_AIRSPEED_MIN_CRSFRATE) || (lastAirspeedCRC != crsfAirspeed.crc))
    {
        lastAirspeedSent = now;
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfAirspeed.h);
    }

    lastAirspeedCRC = crsfAirspeed.crc;
}

#endif
