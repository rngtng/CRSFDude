#include <Arduino.h>
#include "CRSFProtocol.h"

#define CRSF_PIN  20
#define CRSF_BAUD 420000

CRSFProtocol crsf;

static uint32_t lastReportTime = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);
    Serial.println("CRSFDude starting...");

    crsf.begin(CRSF_PIN, CRSF_BAUD);
}

void loop()
{
    bool newChannels = crsf.update();

    // Send telemetry every 5th RC packet
    if (newChannels && crsf.rxPacketCount % 5 == 0) {
        crsf.sendFlightMode("ACRO");
        crsf.sendBattery(111, 15, 1200, 75);          // 11.1V, 1.5A, 1200mAh, 75%
        crsf.sendAttitude(1500, -300, 0);              // pitch 0.15rad, roll -0.03rad
        crsf.sendGPS(524213670, 133560120, 250, 18000, 120, 12); // Berlin, 2.5m/s, 180°, 120m, 12 sats
        crsf.sendBaroAltitude(12034);                  // 120.34m
        crsf.sendVario(150);                           // 1.5m/s climb
    }

    // Report once per second
    uint32_t now = millis();
    if (now - lastReportTime >= 1000) {
        Serial.printf("CH1: %4u  CH2: %4u  CH3: %4u  CH4: %4u  [rx:%u tx:%u /s]\n",
                      crsf.getChannel(0), crsf.getChannel(1),
                      crsf.getChannel(2), crsf.getChannel(3),
                      crsf.rxPacketCount, crsf.txPacketCount);
        crsf.rxPacketCount = crsf.txPacketCount = 0;
        lastReportTime = now;
    }
}
