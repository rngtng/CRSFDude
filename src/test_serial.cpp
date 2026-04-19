#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }  // wait for USB CDC connection
    delay(1000);
    Serial.println("Hello World test starting...");
}

void loop()
{
    Serial.println("Hello World");
    delay(1000);
}
