#include <TinyGPS++.h>
#include <ESPAsyncWebServer.h>

extern AsyncWebServer server;
TimerHandle_t GpsTimer;
HardwareSerial GPSSerial1(1);
TinyGPSPlus gps;

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPSSerial1.available())
      gps.encode(GPSSerial1.read());
  } while (millis() - start < ms);
}

void ShowGPSData()
{
  Serial.println(F("Checking GPS data..."));
  smartDelay(1000);
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS data received: check wiring"));
  }
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");
}

void GPSSetup()
{
  GPSSerial1.begin(9600, SERIAL_8N1, 12, 15); //17-TX 18-RX

  GpsTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(20000), pdTRUE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(ShowGPSData));

  server.on("/gps", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"latitude\":" + String(gps.location.lat(), 5);
    json += ",\"longitude\":" + String(gps.location.lng(), 4);
    json += ",\"satellites\":" + String(gps.satellites.value());
    json += ",\"time\":\"" + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\"";
    json += ",\"Altitude\":" + String(gps.altitude.feet() / 3.2808) + "m";
    json += "}";
    request->send(200, "application/json", json);
    json = String();
  });
  xTimerStart(GpsTimer, 1000);
}
