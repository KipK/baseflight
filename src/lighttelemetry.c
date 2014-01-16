/*
 * LightTelemetry implementation by KipK 
 *
 * This is a lightw & simple one way telemetry protocol that fits in really low baudrates >= 1200 bauds
 * It's targeted for the moment for Ghettostation antenna tracker: https://github.com/KipK/Ghettostation )
 * Will be extended in the future for ground OSD usage.
 */
#include "board.h"
#include "mw.h"

#define CYCLETIME             200


void sendLightTelemetryGPS(void)
{

    uint8_t LTBuff[18];
    //protocol: START(2 bytes)FRAMEID(1byte)LAT(cm,4 bytes)LON(cm,4bytes)SPEED(m/s,2bytes)ALT(cm,4bytes)SATS(6bits)FIX(2bits)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x47; // G ( gps frame at 5hz )
    //PAYLOAD
    LTBuff[3]=(GPS_coord[LAT] >> 8*0) & 0xFF;
    LTBuff[4]=(GPS_coord[LAT] >> 8*1) & 0xFF;
    LTBuff[5]=(GPS_coord[LAT] >> 8*2) & 0xFF;
    LTBuff[6]=(GPS_coord[LAT] >> 8*3) & 0xFF;
    LTBuff[7]=(GPS_coord[LON] >> 8*0) & 0xFF;
    LTBuff[8]=(GPS_coord[LON] >> 8*1) & 0xFF;
    LTBuff[9]=(GPS_coord[LON] >> 8*2) & 0xFF;
    LTBuff[10]=(GPS_coord[LON] >> 8*3) & 0xFF;
    LTBuff[11]=((uint8_t)round(GPS_speed/100) >> 8*0) & 0xFF;
    LTBuff[12]=(BaroAlt >> 8*0) & 0xFF;
    LTBuff[13]=(BaroAlt >> 8*1) & 0xFF;
    LTBuff[14]=(BaroAlt >> 8*2) & 0xFF;
    LTBuff[15]=(BaroAlt >> 8*3) & 0xFF;
    LTBuff[16]= ((GPS_numSat << 2)& 0xFF ) | (f.GPS_FIX & 0b00000011) ; // last 6 bits: sats number, first 2:fix type (0,1,2,3)

    //CRC
    uint8_t LTCrc = 0x00;
    int i;
    for (i = 3; i < 17; i++) {
        LTCrc ^= LTBuff[i];
    }
    LTBuff[17]=LTCrc;

    for (i = 0; i<18; i++) {
        serialWrite(core.mainport,LTBuff[i]);
    }
}

static bool lighttelemetryEnabled = false;

void initLightTelemetry(void)
{

 //nothing usefull yet. Will be added when softserial will work at other baudrates.

}

static uint32_t ltm_lastCycleTime = 0;
static uint8_t ltm_cycleNum = 0;

void updateLightTelemetryState(void)
{
    bool State;
	if (!mcfg.telemetry_switch)
        State = f.ARMED;
    else
        State = rcOptions[BOXTELEMETRY];
	
    if (State != lighttelemetryEnabled) {
        if (State)
            serialInit(mcfg.lighttelemetry_baudrate);
        else
            serialInit(mcfg.serial_baudrate);
        lighttelemetryEnabled = State;
    }
}

void sendLightTelemetry(void)
{
    if ((!mcfg.telemetry_switch && !f.ARMED) || (mcfg.telemetry_switch && !rcOptions[BOXTELEMETRY]))
        return;
    if (serialTotalBytesWaiting(core.mainport) != 0)
        return;
    if (millis() - ltm_lastCycleTime >= CYCLETIME) {
        ltm_lastCycleTime = millis();
        ltm_cycleNum++;
        // Sent every 200ms
        sendLightTelemetryGPS(); // Only one frame at 5hz for now. Will be extended with other frames for ground osd.
    }
}

