#include <Arduino.h>
#include "CRSFDude.h"

#define CRSF_PIN  20
#define CRSF_BAUD 420000

CRSFDude crsf;

static uint32_t lastReportTime = 0;

void onModelChanged(uint8_t id)
{
    Serial.printf("Model switched to %d\n", id);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);
    Serial.println("CRSFDude starting...");

    crsf.begin(CRSF_PIN, CRSF_BAUD);
    crsf.onModelIdChanged = onModelChanged;
}

void loop()
{
    bool newChannels = crsf.update();

    // Send link stats every cycle to keep EdgeTX telemetry streaming active,
    // then rotate one sensor per cycle
    if (newChannels && crsf.rxPacketCount % 5 == 0) {
        // Link stats with non-zero RxQuality — required for sensor discovery
        crsf.sendLinkStats(90, 90, 100, 10, 0, 4, 3, 80, 100, 8);

        static uint8_t telemetrySlot = 0;
        switch (telemetrySlot++ % 6) {
            case 0: crsf.sendFlightMode("ACRO"); break;
            case 1: crsf.sendBattery(111, 15, 1200, 75); break;             // 11.1V, 1.5A, 1200mAh, 75%
            case 2: crsf.sendAttitude(1500, -300, 0); break;                // pitch 0.15rad, roll -0.03rad
            case 3: crsf.sendGPS(524213670, 133560120, 250, 18000, 120, 12); break; // Berlin
            case 4: crsf.sendBaroAltitude(12034); break;                    // 120.34m
            case 5: crsf.sendVario(150); break;                             // 1.5m/s climb
        }
    }

    // Report once per second
    uint32_t now = millis();
    if (now - lastReportTime >= 1000) {
        Serial.printf("CH1: %4u  CH2: %4u  CH3: %4u  CH4: %4u  [rx:%u tx:%u /s] model:%u\n",
                      crsf.getChannel(0), crsf.getChannel(1),
                      crsf.getChannel(2), crsf.getChannel(3),
                      crsf.rxPacketCount, crsf.txPacketCount, crsf.modelId);
        crsf.rxPacketCount = crsf.txPacketCount = 0;
        lastReportTime = now;
    }
}
