/*
 * LightTelemetry implementation by KipK ( used by Ghettostation: https://github.com/KipK/Ghettostation )
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
        serialWrite(core.telemport,LTBuff[i]);
    }
}

void initLightTelemetry(void)
{
//core.telemport = &(softSerialPorts[0].port);
core.telemport = core.mainport;

}

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;

void sendLightTelemetry(void)
{
    if (serialTotalBytesWaiting(core.telemport) != 0)
        return;
    if (millis() - lastCycleTime >= CYCLETIME) {
        lastCycleTime = millis();
        cycleNum++;
        // Sent every 200ms
        sendLightTelemetryGPS();
    }
}

