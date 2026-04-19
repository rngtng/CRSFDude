#include <Arduino.h>
#include "CRSFProtocol.h"

#define CRSF_PIN  20
#define CRSF_BAUD 420000

CRSFProtocol crsf;

static uint32_t lastReportTime = 0;
static uint32_t lastFlightModeToggle = 0;
static bool flightModeOn = false;

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

    // Send flight mode telemetry every 5th RC packet
    if (newChannels && crsf.rxPacketCount % 5 == 0) {
        uint32_t now = millis();
        if (now - lastFlightModeToggle >= 2000) {
            flightModeOn = !flightModeOn;
            lastFlightModeToggle = now;
        }
        crsf.sendFlightMode(flightModeOn ? "ON" : "OFF");
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
